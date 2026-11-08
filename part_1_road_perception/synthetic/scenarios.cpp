#include "synthetic/scenarios.hpp"

#include "common/geometry.hpp"
#include "radar/radar_processor.hpp"
#include "sensor_fusion/radar_fusion.hpp"

#include <algorithm>
#include <cstdio>
#include <random>
#include <sstream>

namespace ipvseas {

namespace {

double heading_of(const ActorState& s) {
    return s.vel.norm() > 0.3 ? std::atan2(s.vel.y(), s.vel.x()) : 0.0;
}

void step_actor(const SyntheticActor& a, double t, double dt, ActorState& s) {
    if (a.velocity_override && a.velocity_override(t, s)) {
        s.pos += s.vel * dt;
        return;
    }
    const Eigen::Vector2d acc = a.accel ? a.accel(t, s) : Eigen::Vector2d::Zero();
    Eigen::Vector2d v_next = s.vel + acc * dt;
    if (s.vel.norm() > 1e-6 && v_next.dot(s.vel) < 0.0) v_next.setZero();  // no reversing
    s.pos += 0.5 * (s.vel + v_next) * dt;
    s.vel = v_next;
}

SyntheticActor car(const std::string& name, double x, double y, double vx, double vy = 0.0) {
    SyntheticActor a;
    a.name = name;
    a.cls = ObjectClass::Car;
    a.size = default_footprint(ObjectClass::Car);
    a.init.pos = {x, y};
    a.init.vel = {vx, vy};
    return a;
}

}  // namespace

std::vector<Scenario> standard_scenarios() {
    std::vector<Scenario> v;

    {   // S1: lead vehicle brakes hard
        Scenario s;
        s.name = "S1_lead_vehicle_hard_braking";
        s.description = "Lead car 40 m ahead at 90 km/h brakes at 6 m/s^2 from t = 2 s";
        s.ego_speed = 25.0;
        SyntheticActor a = car("lead", 40.0, 0.0, 25.0);
        a.accel = [](double t, const ActorState&) {
            return t >= 2.0 ? Eigen::Vector2d(-6.0, 0.0) : Eigen::Vector2d::Zero();
        };
        s.actors.push_back(a);
        v.push_back(s);
    }
    {   // S2: stationary broken-down truck
        Scenario s;
        s.name = "S2_stationary_truck";
        s.description = "Broken-down truck 150 m ahead, ego at 90 km/h, sensor range 100 m";
        s.ego_speed = 25.0;
        SyntheticActor a;
        a.name = "truck";
        a.cls = ObjectClass::Truck;
        a.size = default_footprint(ObjectClass::Truck);
        a.init.pos = {150.0, 0.0};
        s.actors.push_back(a);
        v.push_back(s);
    }
    {   // S3: much slower vehicle ahead
        Scenario s;
        s.name = "S3_slow_vehicle_ahead";
        s.description = "Tractor-like vehicle at 29 km/h, 70 m ahead, ego at 90 km/h";
        s.ego_speed = 25.0;
        s.actors.push_back(car("slow", 70.0, 0.0, 8.0));
        v.push_back(s);
    }
    {   // S4: cut-in from the adjacent lane
        Scenario s;
        s.name = "S4_cut_in";
        s.description = "Car in the left lane, 30 m ahead at 72 km/h, merges into ego lane at 1 m/s";
        s.ego_speed = 25.0;
        SyntheticActor a = car("cutter", 30.0, 3.5, 20.0);
        a.velocity_override = [](double t, ActorState& st) {
            if (t < 1.5) return false;
            st.vel = {20.0, st.pos.y() > 0.0 ? -1.0 : 0.0};
            return true;
        };
        s.actors.push_back(a);
        s.min_lead_time = 1.0;
        v.push_back(s);
    }
    {   // S5: pedestrian steps out from behind a parked vehicle
        Scenario s;
        s.name = "S5_pedestrian_emerging";
        s.description = "Pedestrian occluded until 3.5 m from the ego centreline, crosses at 1.4 m/s; ego 43 km/h";
        s.ego_speed = 12.0;
        SyntheticActor p;
        p.name = "pedestrian";
        p.cls = ObjectClass::Pedestrian;
        p.size = default_footprint(ObjectClass::Pedestrian);
        p.init.pos = {45.0, -4.5};
        p.velocity_override = [](double t, ActorState& st) {
            st.vel = t >= 0.5 ? Eigen::Vector2d(0.0, 1.4) : Eigen::Vector2d::Zero();
            return true;
        };
        p.visible = [](double, const ActorState& st) { return st.pos.y() > -3.5; };
        s.actors.push_back(p);
        s.min_lead_time = 1.0;
        v.push_back(s);
    }
    {   // S6: animal crossing a highway
        Scenario s;
        s.name = "S6_animal_crossing";
        s.description = "Cow 110 m ahead walking across at 0.8 m/s, ego at 79 km/h";
        s.ego_speed = 22.0;
        SyntheticActor c;
        c.name = "cow";
        c.cls = ObjectClass::Animal;
        c.size = default_footprint(ObjectClass::Animal);
        c.init.pos = {110.0, -4.0};
        c.velocity_override = [](double, ActorState& st) {
            st.vel = {0.0, 0.8};
            return true;
        };
        s.actors.push_back(c);
        v.push_back(s);
    }

    {   // S7: stationary truck at expressway speed: beyond LiDAR range, radar must see it
        Scenario s;
        s.name = "S7_stationary_truck_120kmh";
        s.description = "Broken-down truck 230 m ahead, ego at 120 km/h; LiDAR 100 m, radar 200 m";
        s.ego_speed = 33.3;
        SyntheticActor a;
        a.name = "truck";
        a.cls = ObjectClass::Truck;
        a.size = default_footprint(ObjectClass::Truck);
        a.init.pos = {230.0, 0.0};
        s.actors.push_back(a);
        // A 1.2 s / 6 m/s^2 driver needs ~4.0 s at 120 km/h. With 0.5 deg elevation accuracy,
        // radar can only confirm the truck is not a manhole cover / bridge from ~110 m, which
        // caps the achievable lead time near 3 s: a documented sensor limitation (see spec).
        s.min_lead_time = 2.8;
        v.push_back(s);
    }

    // ---------------- false-alarm tests: must NOT reach WARNING ----------------
    {
        Scenario s;
        s.name = "F1_overtaking_car";
        s.description = "Car overtakes in the left lane (3.5 m) at 108 km/h";
        s.ego_speed = 25.0;
        s.actors.push_back(car("overtaker", 5.0, 3.5, 30.0));
        s.expect_collision = false;
        v.push_back(s);
    }
    {
        Scenario s;
        s.name = "F2_pedestrian_on_footpath";
        s.description = "Pedestrian walks along the footpath 5 m to the right";
        s.ego_speed = 12.0;
        SyntheticActor p;
        p.name = "walker";
        p.cls = ObjectClass::Pedestrian;
        p.size = default_footprint(ObjectClass::Pedestrian);
        p.init.pos = {40.0, -5.0};
        p.init.vel = {1.4, 0.0};
        s.actors.push_back(p);
        s.expect_collision = false;
        v.push_back(s);
    }
    {
        Scenario s;
        s.name = "F3_steady_following";
        s.description = "Following a car 30 m ahead at the same speed (1.2 s headway)";
        s.ego_speed = 25.0;
        s.actors.push_back(car("lead", 30.0, 0.0, 25.0));
        s.expect_collision = false;
        v.push_back(s);
    }
    {
        Scenario s;
        s.name = "F4_parked_cars_roadside";
        s.description = "Cars parked 3.0 m to the right of the ego centreline (1.1 m clearance)";
        s.ego_speed = 12.0;
        for (double x : {40.0, 60.0, 80.0}) s.actors.push_back(car("parked", x, -3.0, 0.0));
        s.expect_collision = false;
        v.push_back(s);
    }
    {
        Scenario s;
        s.name = "F5_oncoming_traffic";
        s.description = "Oncoming car in the opposite lane (3.5 m) at 72 km/h";
        s.ego_speed = 20.0;
        s.actors.push_back(car("oncoming", 120.0, 3.5, -20.0));
        s.expect_collision = false;
        v.push_back(s);
    }
    {
        Scenario s;
        s.name = "F6_overhead_gantry";
        s.description = "Sign gantry 5.5-6.5 m above the road, 140 m ahead, ego at 100 km/h (radar sees it)";
        s.ego_speed = 27.8;
        RadarStructure g;
        g.name = "gantry";
        g.pos = {140.0, 0.0};
        s.structures.push_back(g);
        s.expect_collision = false;
        v.push_back(s);
    }
    {
        Scenario s;
        s.name = "F7_manhole_cover";
        s.description = "Metal plate flush with the road 90 m ahead, ego at 72 km/h (radar sees it)";
        s.ego_speed = 20.0;
        RadarStructure m;
        m.name = "manhole";
        m.pos = {90.0, 0.0};
        m.length = 0.8;
        m.width = 0.8;
        m.z_bottom = 0.0;
        m.z_top = 0.05;
        s.structures.push_back(m);
        s.expect_collision = false;
        v.push_back(s);
    }
    return v;
}

ScenarioResult run_scenario(const Scenario& sc, const Part1Config& cfg) {
    ScenarioResult r;
    r.name = sc.name;
    r.expect_collision = sc.expect_collision;
    r.min_lead_time = sc.min_lead_time;

    // ---- ground truth (inattentive driver: constant speed, straight) ----
    const int n = static_cast<int>(std::round(sc.duration / sc.dt));
    const EgoState ego_proto;
    std::vector<std::vector<ActorState>> gt(n + 1, std::vector<ActorState>(sc.actors.size()));
    std::vector<ActorState> st(sc.actors.size());
    for (size_t i = 0; i < sc.actors.size(); ++i) st[i] = sc.actors[i].init;
    for (int k = 0; k <= n; ++k) {
        const double t = k * sc.dt;
        if (k > 0)
            for (size_t i = 0; i < sc.actors.size(); ++i) step_actor(sc.actors[i], t, sc.dt, st[i]);
        gt[k] = st;
        if (!std::isfinite(r.t_collision)) {
            const Eigen::Vector2d ego_p(sc.ego_speed * t, 0.0);
            for (size_t i = 0; i < sc.actors.size(); ++i)
                if (footprints_overlap(ego_p, 0.0, ego_proto.length, ego_proto.width, st[i].pos,
                                       heading_of(st[i]), sc.actors[i].size.x(), sc.actors[i].size.y())) {
                    r.t_collision = t;
                    break;
                }
        }
    }

    // ---- run the pipeline on noisy detections ----
    Part1Pipeline pipe(cfg);
    std::mt19937 rng(sc.sensor.seed);
    std::normal_distribution<double> N01(0.0, 1.0);
    std::uniform_real_distribution<double> U01(0.0, 1.0);
    double ttc_err_sum = 0.0;
    std::ostringstream csv;
    csv << "t,ego_x,level,track_id,ttc,p_collision,true_remaining\n";

    for (int k = 0; k <= n; ++k) {
        const double t = k * sc.dt;
        if (std::isfinite(r.t_collision) && t > r.t_collision) break;  // stop at impact
        EgoState ego;
        ego.t = t;
        ego.x = sc.ego_speed * t;
        ego.speed = sc.ego_speed;

        // ---- primary sensor (camera + LiDAR, already fused), VEHICLE frame ----
        std::vector<Detection> primary;
        for (size_t i = 0; i < sc.actors.size(); ++i) {
            const SyntheticActor& a = sc.actors[i];
            const ActorState& s = gt[k][i];
            const Eigen::Vector2d rel = s.pos - Eigen::Vector2d(ego.x, ego.y);
            const double range = rel.norm();
            if (rel.x() < sc.sensor.min_x || range > sc.sensor.max_range) continue;
            if (a.visible && !a.visible(t, s)) continue;
            if (U01(rng) > sc.sensor.detect_prob) continue;
            const double sig = sc.sensor.pos_sigma_base + sc.sensor.pos_sigma_per_m * range;
            Detection d;
            d.t = t;
            d.pos = rel + Eigen::Vector2d(N01(rng), N01(rng)) * sig;
            d.cov = Eigen::Matrix2d::Identity() * sig * sig;
            d.cls = a.cls;
            d.confidence = 0.9;
            d.length = a.size.x();
            d.width = a.size.y();
            d.yaw = heading_of(s);
            d.source = Sensor::Lidar;
            primary.push_back(d);
        }
        if (U01(rng) < sc.sensor.clutter_rate) {
            Detection c;
            c.t = t;
            c.pos = {5.0 + 75.0 * U01(rng), -12.0 + 24.0 * U01(rng)};
            c.cov = Eigen::Matrix2d::Identity() * 0.25;
            c.cls = ObjectClass::Unknown;
            c.confidence = 0.3;
            primary.push_back(c);
        }

        // ---- 4D imaging radar ----
        std::vector<Detection> vehicle_dets = primary;
        if (sc.sensor.radar) {
            const RadarMount mount;
            const RadarConfig rcfg;
            const Eigen::Vector2d vs = radar_sensor_velocity(mount, ego.speed, 0.0);
            std::vector<RadarPoint> pts;
            // One return from vehicle-frame point q (z above road) moving with world velocity v.
            auto add_point = [&](const Eigen::Vector3d& q, const Eigen::Vector2d& v, double extra_dv) {
                const Eigen::Vector3d d = q - Eigen::Vector3d(mount.x, mount.y, mount.z);
                const double dist = d.norm();
                const double az = std::atan2(d.y(), d.x());
                if (dist > sc.sensor.radar_range || std::abs(az) > sc.sensor.radar_half_fov || d.x() <= 0.5) return;
                const double el = std::asin(d.z() / dist);
                const Eigen::Vector3d u = d / dist;
                const double rr = (v - vs).dot(u.head<2>()) + extra_dv;
                pts.push_back({float(dist + N01(rng) * rcfg.sigma_range), float(az + N01(rng) * rcfg.sigma_azimuth),
                               float(el + N01(rng) * rcfg.sigma_elevation),
                               float(rr + N01(rng) * rcfg.sigma_range_rate), 0.0f});
            };
            for (size_t i = 0; i < sc.actors.size(); ++i) {
                const SyntheticActor& a = sc.actors[i];
                const ActorState& s = gt[k][i];
                if (a.visible && !a.visible(t, s)) continue;
                const Eigen::Vector2d rel = s.pos - Eigen::Vector2d(ego.x, ego.y);
                const double range = std::max(5.0, rel.norm());
                const double h = default_height(a.cls);
                const double size_factor = std::sqrt(a.size.y() * h / (1.9 * 1.5));
                const int n_exp = std::max(1, static_cast<int>(std::round(
                                                  sc.sensor.radar_points_at_50m * size_factor * 50.0 / range)));
                const bool vru = group_of(a.cls) == ClassGroup::VulnerableRoadUser || a.cls == ObjectClass::Animal;
                for (int p = 0; p < n_exp; ++p) {
                    if (U01(rng) > sc.sensor.radar_point_prob) continue;
                    // Returns spread over the footprint, width and height. (Simplification: the
                    // synthetic LiDAR reports box centres, so radar centroids are kept consistent;
                    // real LiDAR and radar both see the near face, see fuse_radar.)
                    const Eigen::Vector3d q(rel.x() + (U01(rng) - 0.5) * std::min(a.size.x(), 2.0),
                                            rel.y() + (U01(rng) - 0.5) * a.size.y(), 0.3 + U01(rng) * (h - 0.3));
                    add_point(q, s.vel, vru ? N01(rng) * 0.4 : 0.0);  // limbs: micro-Doppler
                }
                if (U01(rng) < sc.sensor.ghost_prob)  // multipath via the road: appears below it
                    add_point({rel.x(), rel.y(), -0.9}, s.vel, 0.0);
            }
            for (const RadarStructure& structure : sc.structures) {
                const RadarStructure& rs = structure;
                const Eigen::Vector2d rel = rs.pos - Eigen::Vector2d(ego.x, ego.y);
                const double range = std::max(5.0, rel.norm());
                const int n_exp = std::max(1, static_cast<int>(std::round(
                                                  sc.sensor.radar_points_at_50m * 50.0 / range *
                                                  std::sqrt(rs.width * (rs.z_top - rs.z_bottom + 0.1) / 2.85))));
                for (int p = 0; p < n_exp; ++p) {
                    if (U01(rng) > sc.sensor.radar_point_prob) continue;
                    add_point({rel.x(), rel.y() + (U01(rng) - 0.5) * rs.width,
                               rs.z_bottom + U01(rng) * (rs.z_top - rs.z_bottom)},
                              Eigen::Vector2d::Zero(), 0.0);
                }
            }
            for (int g = 0; g < sc.sensor.guardrail_points; ++g)  // static road-edge clutter
                add_point({10.0 + 140.0 * U01(rng), U01(rng) < 0.5 ? 7.0 : -7.0, 0.3 + 0.6 * U01(rng)},
                          Eigen::Vector2d::Zero(), 0.0);

            std::vector<Detection> radar_dets;
            for (const RadarObject& o : process_radar(pts, mount, rcfg, ego.speed, 0.0))
                if (radar_object_usable(o, mount, rcfg))
                    radar_dets.push_back(radar_object_to_detection(o, mount, rcfg, ego.speed, 0.0, t));
            vehicle_dets = fuse_radar(primary, radar_dets);
        }

        std::vector<Detection> dets;
        for (const Detection& d : vehicle_dets) dets.push_back(vehicle_to_world(d, ego));

        const Part1Output out = pipe.step(t, ego, dets);
        const WarningLevel lvl = out.warning.level;
        if (lvl > r.max_level) r.max_level = lvl;
        if (lvl >= WarningLevel::Caution && !std::isfinite(r.t_first_caution)) r.t_first_caution = t;
        if (lvl >= WarningLevel::Warning && !std::isfinite(r.t_first_warning)) r.t_first_warning = t;
        if (lvl >= WarningLevel::Critical && !std::isfinite(r.t_first_critical)) r.t_first_critical = t;
        if (lvl >= WarningLevel::Warning) ++r.frames_at_warning_or_above;

        double best_ttc = kInf, best_p = 0.0;
        int best_id = -1;
        for (const Hazard& h : out.hazards)
            if (h.ttc < best_ttc) { best_ttc = h.ttc; best_p = h.collision_probability; best_id = h.track_id; }
        const double remaining = std::isfinite(r.t_collision) ? r.t_collision - t : kInf;
        if (std::isfinite(best_ttc) && remaining > 0.0 && remaining <= 5.0) {
            ttc_err_sum += std::abs(best_ttc - remaining);
            ++r.ttc_samples;
        }
        char line[160];
        std::snprintf(line, sizeof(line), "%.2f,%.2f,%d,%d,%.3f,%.3f,%.3f\n", t, ego.x,
                      static_cast<int>(lvl), best_id, std::isfinite(best_ttc) ? best_ttc : -1.0, best_p,
                      std::isfinite(remaining) ? remaining : -1.0);
        csv << line;
    }

    if (r.ttc_samples > 0) r.ttc_mae = ttc_err_sum / r.ttc_samples;
    if (std::isfinite(r.t_collision) && std::isfinite(r.t_first_warning))
        r.lead_time = r.t_collision - r.t_first_warning;
    r.passed = sc.expect_collision
                   ? (std::isfinite(r.t_collision) && r.lead_time >= sc.min_lead_time)
                   : (r.frames_at_warning_or_above == 0);
    r.csv = csv.str();
    return r;
}

}  // namespace ipvseas
