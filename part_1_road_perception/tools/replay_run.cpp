// Replays a recorded run (CARLA via simulation/carla/record_run.py, or the synthetic
// generator make_synthetic_run.py) through the full Part 1 pipeline:
//   LiDAR obstacles + YOLO boxes -> camera-LiDAR fusion -> world frame -> tracking
//   -> prediction -> collision risk -> warnings
//
//   usage: part1_replay <run_dir> [--lidar-only] [--no-radar] [--tag NAME]
//     --tag NAME  write part1_output_NAME.csv / tracks_NAME.csv (keeps several sensor
//                 configurations of the same recording side by side)
//
// Inputs (see docs/part1_specification.md, "Run folder format"):
//   calib.txt, ego.csv, lidar/<frame>.bin, camera_dets.csv (optional), radar/<frame>.bin (optional)
// Outputs (written into <run_dir>):
//   part1_output.csv : frame,t,level,track_id,ttc,p_collision,num_confirmed
//   tracks.csv       : frame,track_id,cls,x,y,vx,vy,mature
#include <algorithm>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "common/geometry.hpp"
#include "lidar/obstacle_detector.hpp"
#include "part1_pipeline.hpp"
#include "radar/radar_processor.hpp"
#include "sensor_fusion/radar_fusion.hpp"
#include "sensor_fusion/camera_lidar_fusion.hpp"

using namespace ipvseas;

namespace {

std::map<std::string, double> read_calib(const std::string& path) {
    std::map<std::string, double> kv;
    std::ifstream f(path);
    std::string line;
    while (std::getline(f, line)) {
        const auto eq = line.find('=');
        if (line.empty() || line[0] == '#' || eq == std::string::npos) continue;
        kv[line.substr(0, eq)] = std::stod(line.substr(eq + 1));
    }
    return kv;
}

std::vector<std::vector<std::string>> read_csv(const std::string& path) {
    std::vector<std::vector<std::string>> rows;
    std::ifstream f(path);
    std::string line;
    bool header = true;
    while (std::getline(f, line)) {
        if (header) { header = false; continue; }
        if (line.empty()) continue;
        std::vector<std::string> cols;
        std::stringstream ss(line);
        std::string c;
        while (std::getline(ss, c, ',')) cols.push_back(c);
        rows.push_back(cols);
    }
    return rows;
}

std::vector<LidarPoint> read_lidar(const std::string& path) {
    std::vector<LidarPoint> pts;
    std::ifstream f(path, std::ios::binary);
    if (!f) return pts;
    f.seekg(0, std::ios::end);
    const auto bytes = static_cast<size_t>(f.tellg());
    f.seekg(0);
    pts.resize(bytes / sizeof(LidarPoint));
    f.read(reinterpret_cast<char*>(pts.data()), static_cast<std::streamsize>(pts.size() * sizeof(LidarPoint)));
    return pts;
}

std::vector<RadarPoint> read_radar(const std::string& path) {
    std::vector<RadarPoint> pts;
    std::ifstream f(path, std::ios::binary);
    if (!f) return pts;
    f.seekg(0, std::ios::end);
    const auto bytes = static_cast<size_t>(f.tellg());
    f.seekg(0);
    pts.resize(bytes / sizeof(RadarPoint));
    f.read(reinterpret_cast<char*>(pts.data()), static_cast<std::streamsize>(pts.size() * sizeof(RadarPoint)));
    return pts;
}

std::string frame_name(int frame) {
    char b[16];
    std::snprintf(b, sizeof(b), "%06d", frame);
    return b;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: part1_replay <run_dir> [--lidar-only] [--no-radar] [--tag NAME]\n";
        return 2;
    }
    const std::string run = argv[1];
    bool lidar_only = false, use_radar = true;
    std::string tag;
    for (int i = 2; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--lidar-only") lidar_only = true;
        else if (a == "--no-radar") use_radar = false;
        else if (a == "--tag" && i + 1 < argc) tag = argv[++i];
    }
    const std::string suffix = tag.empty() ? "" : "_" + tag;

    const auto calib = read_calib(run + "/calib.txt");
    auto get = [&](const char* k, double def) {
        const auto it = calib.find(k);
        return it == calib.end() ? def : it->second;
    };
    const CameraModel cam = CameraModel::from_fov(
        static_cast<int>(get("cam_width", 1280)), static_cast<int>(get("cam_height", 720)),
        get("cam_fov", 90.0), {get("cam_x", 1.5), get("cam_y", 0.0), get("cam_z", 1.6)},
        get("cam_pitch_deg", 0.0) * kPi / 180.0);

    RadarMount mount;
    mount.x = get("radar_x", mount.x);
    mount.y = get("radar_y", mount.y);
    mount.z = get("radar_z", mount.z);
    mount.yaw = get("radar_yaw_deg", 0.0) * kPi / 180.0;
    const double radar_vsign = get("radar_velocity_sign", 1.0);
    const RadarConfig rcfg;

    const auto ego_rows = read_csv(run + "/ego.csv");
    if (ego_rows.empty()) {
        std::cerr << "no ego.csv rows in " << run << "\n";
        return 1;
    }
    std::map<int, std::vector<CameraDetection>> cam_dets;
    if (!lidar_only)
        for (const auto& r : read_csv(run + "/camera_dets.csv")) {
            if (r.size() < 7) continue;
            CameraDetection d;
            d.box = {std::stod(r[1]), std::stod(r[2]), std::stod(r[3]), std::stod(r[4])};
            d.cls = class_from_string(r[5]);
            d.confidence = std::stod(r[6]);
            cam_dets[std::stoi(r[0])].push_back(d);
        }

    Part1Pipeline pipe;
    const LidarConfig lcfg;
    const FusionConfig fcfg;
    std::ofstream out(run + "/part1_output" + suffix + ".csv"), trk(run + "/tracks" + suffix + ".csv");
    out << "frame,t,level,track_id,ttc,p_collision,num_confirmed\n";
    trk << "frame,track_id,cls,x,y,vx,vy,mature\n";

    int max_level = 0;
    for (const auto& r : ego_rows) {
        if (r.size() < 8) continue;
        const int frame = std::stoi(r[0]);
        EgoState ego;
        ego.t = std::stod(r[1]);
        ego.x = std::stod(r[2]);
        ego.y = std::stod(r[3]);
        ego.yaw = std::stod(r[4]);
        ego.speed = std::stod(r[5]);
        ego.yaw_rate = std::stod(r[6]);
        ego.accel = std::stod(r[7]);

        const auto cloud = read_lidar(run + "/lidar/" + frame_name(frame) + ".bin");
        const auto obstacles = detect_obstacles(cloud, lcfg);
        const auto it = cam_dets.find(frame);
        const std::vector<CameraDetection> no_dets;
        auto vehicle_dets =
            fuse_camera_lidar(cam, it == cam_dets.end() ? no_dets : it->second, obstacles, cloud, fcfg, ego.t);
        if (use_radar) {
            auto rpts = read_radar(run + "/radar/" + frame_name(frame) + ".bin");
            for (auto& p : rpts) p.range_rate = static_cast<float>(p.range_rate * radar_vsign);
            std::vector<Detection> radar_dets;
            for (const RadarObject& o : process_radar(rpts, mount, rcfg, ego.speed, ego.yaw_rate))
                if (radar_object_usable(o, mount, rcfg))
                    radar_dets.push_back(radar_object_to_detection(o, mount, rcfg, ego.speed, ego.yaw_rate, ego.t));
            vehicle_dets = fuse_radar(vehicle_dets, radar_dets);
        }
        std::vector<Detection> world;
        for (const auto& d : vehicle_dets) world.push_back(vehicle_to_world(d, ego));

        const Part1Output o = pipe.step(ego.t, ego, world);
        const Hazard* top = o.hazards.empty() ? nullptr : &o.hazards.front();
        out << frame << ',' << ego.t << ',' << static_cast<int>(o.warning.level) << ','
            << o.warning.track_id << ',' << (top && std::isfinite(top->ttc) ? top->ttc : -1.0) << ','
            << (top ? top->collision_probability : 0.0) << ',' << pipe.tracker().confirmed_tracks().size()
            << '\n';
        for (const Track* tr : pipe.tracker().confirmed_tracks())
            trk << frame << ',' << tr->id << ',' << to_string(tr->cls) << ',' << tr->kf.pos().x() << ','
                << tr->kf.pos().y() << ',' << tr->kf.vel().x() << ',' << tr->kf.vel().y() << ','
                << (pipe.tracker().is_mature(*tr) ? 1 : 0) << '\n';
        max_level = std::max(max_level, static_cast<int>(o.warning.level));
    }
    std::printf("replayed %zu frames from %s, max warning level %s\n", ego_rows.size(), run.c_str(),
                to_string(static_cast<WarningLevel>(max_level)));
    std::printf("wrote %s/part1_output%s.csv and %s/tracks%s.csv\n", run.c_str(), suffix.c_str(), run.c_str(),
                suffix.c_str());
    return 0;
}
