// Unit and scenario tests for IPVSEAS Part 1 (no external test framework needed).
#include <algorithm>
#include <cstdio>
#include <functional>
#include <random>
#include <string>
#include <vector>

#include "behaviour_prediction/trajectory_predictor.hpp"
#include "collision_prediction/collision_risk.hpp"
#include "collision_prediction/warning_manager.hpp"
#include "common/geometry.hpp"
#include "lidar/obstacle_detector.hpp"
#include "radar/radar_processor.hpp"
#include "sensor_fusion/radar_fusion.hpp"
#include "sensor_fusion/camera_lidar_fusion.hpp"
#include "synthetic/scenarios.hpp"
#include "tracking/hungarian.hpp"
#include "tracking/tracker.hpp"

using namespace ipvseas;

static int g_failures = 0;
#define CHECK(cond)                                                                 \
    do {                                                                            \
        if (!(cond)) {                                                              \
            std::printf("    CHECK FAILED: %s  (%s:%d)\n", #cond, __FILE__, __LINE__); \
            ++g_failures;                                                           \
        }                                                                           \
    } while (0)
#define CHECK_NEAR(a, b, tol) CHECK(std::abs((a) - (b)) <= (tol))

// ----------------------------------------------------------------- tracking
static void test_hungarian() {
    Eigen::MatrixXd c(3, 3);
    c << 4, 1, 3,
         2, 0, 5,
         3, 2, 2;
    const auto a = hungarian_assign(c, 100.0);
    CHECK(a[0] == 1 && a[1] == 0 && a[2] == 2);  // optimal total cost 5

    Eigen::MatrixXd r(2, 3);  // rectangular, one forbidden pair
    r << 1, 100, 100,
         100, 100, 2;
    const auto b = hungarian_assign(r, 50.0);
    CHECK(b[0] == 0 && b[1] == 2);
}

static void test_kalman_converges() {
    // Object moving at (10, -2) m/s with noisy position measurements at 20 Hz.
    KalmanCA kf({0, 0}, Eigen::Matrix2d::Identity() * 0.04, 15.0, 3.0, 2.0);
    std::mt19937 rng(3);
    std::normal_distribution<double> n(0.0, 0.2);
    for (int k = 1; k <= 60; ++k) {
        const double t = k * 0.05;
        kf.predict(0.05);
        kf.update({10.0 * t + n(rng), -2.0 * t + n(rng)}, Eigen::Matrix2d::Identity() * 0.04);
    }
    CHECK_NEAR(kf.vel().x(), 10.0, 0.6);
    CHECK_NEAR(kf.vel().y(), -2.0, 0.6);
}

static void test_tracker_keeps_ids() {
    // Two crossing objects must keep their identities.
    MultiObjectTracker trk;
    for (int k = 0; k < 40; ++k) {
        const double t = k * 0.05;
        Detection a, b;
        a.t = b.t = t;
        a.pos = {20.0 + 10.0 * t, 1.0};
        b.pos = {40.0 - 10.0 * t, -1.0};
        a.cls = b.cls = ObjectClass::Car;
        trk.update(t, {a, b});
    }
    const auto conf = trk.confirmed_tracks();
    CHECK(conf.size() == 2);
    for (const Track* tr : conf) {
        // Track id 1 started at x=20 moving +x; id 2 moving -x.
        if (tr->id == 1) CHECK(tr->kf.vel().x() > 8.0);
        if (tr->id == 2) CHECK(tr->kf.vel().x() < -8.0);
    }
}

static void test_tracker_rejects_clutter() {
    MultiObjectTracker trk;
    std::mt19937 rng(5);
    std::uniform_real_distribution<double> u(0.0, 80.0);
    for (int k = 0; k < 40; ++k) {
        Detection d;
        d.t = k * 0.05;
        d.pos = {u(rng), u(rng) - 40.0};  // random position every frame
        trk.update(d.t, {d});
    }
    CHECK(trk.confirmed_tracks().empty());
}

// --------------------------------------------------------------- prediction
static void test_prediction_stops_braking_vehicle() {
    Track tr;
    tr.cls = ObjectClass::Car;
    Vec6 x;
    x << 0, 0, 20, 0, -8, 0;  // 20 m/s braking at 8 m/s^2 -> stops after 25 m
    PredictionConfig pc;
    const Trajectory traj = rollout_state(x, tr.cls, pc, 0.0);
    CHECK_NEAR(traj.back().pos.x(), 25.0, 0.3);
    CHECK_NEAR(traj.back().vel.norm(), 0.0, 1e-9);
}

// ------------------------------------------------------------------- risk
static void test_ttc_formula() {
    CHECK_NEAR(time_to_collision_ca(50.0, -10.0, 0.0), 5.0, 1e-9);
    // Lead decelerates relative to us: gap 20, v_rel = 0, a_rel = -2  ->  t = sqrt(20) ~ 4.47
    CHECK_NEAR(time_to_collision_ca(20.0, 0.0, -2.0), std::sqrt(20.0), 1e-6);
    CHECK(!std::isfinite(time_to_collision_ca(20.0, 5.0, 0.0)));  // opening gap
}

static void test_footprint_overlap() {
    CHECK(footprints_overlap({0, 0}, 0, 4.7, 1.9, {4.0, 0.5}, 0.3, 4.5, 1.9));
    CHECK(!footprints_overlap({0, 0}, 0, 4.7, 1.9, {0.0, 3.5}, 0.0, 4.5, 1.9));  // adjacent lane
    CHECK(!footprints_overlap({0, 0}, 0, 4.7, 1.9, {6.0, 0.0}, 0.0, 4.5, 1.9));  // 1.4 m gap
}

static void test_braking_rollout_matches_stopping_distance() {
    // Driver reacts after 1.2 s, then brakes at 6 m/s^2 from 25 m/s:
    // stopping distance = 25*1.2 + 25^2/(2*6) = 30 + 52.08 = 82.08 m
    EgoState ego;
    ego.speed = 25.0;
    PredictionConfig pc;
    pc.horizon = 8.0;
    const Trajectory b = predict_ego_braking(ego, pc, 1.2, 6.0);
    CHECK_NEAR(b.back().pos.x(), 82.08, 0.5);
    CHECK_NEAR(b.back().vel.norm(), 0.0, 1e-9);

    // Stationary obstacle: WARNING exactly when the driver can no longer stop in time.
    RiskConfig rc;
    CollisionRiskAssessor ra(rc, PredictionConfig{});
    const auto plans = ra.plan_ego(ego);
    auto obstacle_at = [&](double x) {
        Track tr;
        tr.id = 1;
        tr.cls = ObjectClass::Truck;
        tr.confirmed = true;
        tr.kf = KalmanCA({x, 0.0}, Eigen::Matrix2d::Identity() * 0.01, 0.05, 0.05, 1.0);
        return ra.assess_track(tr, ego, plans, true);
    };
    // Gap needed = 82.08 + half lengths (2.35 + 4.5) + margins (0.6) ~ 89.5 m
    CHECK(obstacle_at(95.0).driver_can_avoid);
    CHECK(!obstacle_at(85.0).driver_can_avoid);
    CHECK(obstacle_at(85.0).level == WarningLevel::Warning);
    CHECK(obstacle_at(40.0).level == WarningLevel::Critical);
}

static void test_warning_hysteresis() {
    WarningManager wm;
    Hazard h;
    h.track_id = 7;
    h.level = WarningLevel::Warning;
    CHECK(wm.update({h}).level == WarningLevel::None);     // 1 frame: not yet
    CHECK(wm.update({h}).level == WarningLevel::None);     // 2 frames: not yet
    CHECK(wm.update({h}).level == WarningLevel::Warning);  // 3 frames (0.15 s): raised
    for (int i = 0; i < 9; ++i) CHECK(wm.update({}).level == WarningLevel::Warning);  // held
    CHECK(wm.update({}).level == WarningLevel::None);      // lowered after 10 quiet frames
    Hazard c = h;
    c.level = WarningLevel::Critical;
    CHECK(wm.update({c}).level == WarningLevel::Critical);  // critical raises immediately
}

// ------------------------------------------------------------------ lidar
static std::vector<LidarPoint> synthetic_cloud() {
    std::vector<LidarPoint> pts;
    std::mt19937 rng(11);
    std::normal_distribution<double> n(0.0, 0.03);
    // Flat road at z = -0.0 (vehicle frame origin on the ground)
    for (double x = 2; x < 60; x += 0.5)
        for (double y = -10; y <= 10; y += 0.5) pts.push_back({float(x), float(y), float(n(rng)), 0.2f});
    // Car centred at (20, 2), 4.5 x 1.9 x 1.5. A real LiDAR sees an "L": the rear face and
    // the side facing the sensor (y = 1.05), not the far side.
    for (double z = 0.3; z <= 1.5; z += 0.15) {
        for (double x = 17.75; x <= 22.25; x += 0.15) pts.push_back({float(x), 1.05f, float(z), 0.5f});
        for (double y = 1.05; y <= 2.95; y += 0.15) pts.push_back({17.75f, float(y), float(z), 0.5f});
    }
    // Small unknown object (debris) at (35, -1): 0.6 x 0.6 x 0.5
    for (double x = 34.7; x <= 35.3; x += 0.1)
        for (double y = -1.3; y <= -0.7; y += 0.1) pts.push_back({float(x), float(y), 0.5f, 0.4f});
    // Overhead sign gantry at 6 m height across the road: must be ignored
    for (double y = -10; y <= 10; y += 0.2) pts.push_back({45.0f, float(y), 6.0f, 0.6f});
    return pts;
}

static void test_lidar_ground_and_clusters() {
    LidarConfig cfg;
    GroundPlane g;
    const auto obs = detect_obstacles(synthetic_cloud(), cfg, &g);
    CHECK(g.normal.z() > 0.99);
    CHECK_NEAR(g.d, 0.0, 0.05);
    CHECK(obs.size() == 2);  // car + debris; gantry and road removed
    if (obs.size() == 2) {
        CHECK_NEAR(obs[0].center.x(), 20.0, 0.3);
        CHECK_NEAR(obs[0].center.y(), 2.0, 0.3);
        CHECK_NEAR(obs[0].length, 4.5, 0.4);
        CHECK_NEAR(obs[1].center.x(), 35.0, 0.3);
    }
}

// ----------------------------------------------------------------- fusion
static void test_camera_projection_and_fusion() {
    const CameraModel cam = CameraModel::from_fov(1280, 720, 90.0, {1.5, 0.0, 1.6});
    Eigen::Vector2d uv;
    CHECK(cam.project({21.5, 0.0, 1.6}, uv));      // straight ahead at camera height
    CHECK_NEAR(uv.x(), 640.0, 1e-6);
    CHECK_NEAR(uv.y(), 360.0, 1e-6);
    CHECK(cam.project({11.5, 2.0, 1.6}, uv));      // left of centre -> smaller u
    CHECK(uv.x() < 640.0);

    // Mono range: box bottom on the ground 20 m ahead of the camera.
    Eigen::Vector2d ground_uv;
    cam.project({21.5, 0.0, 0.0}, ground_uv);
    CameraDetection cd{{ground_uv.x() - 40, ground_uv.y() - 60, ground_uv.x() + 40, ground_uv.y()},
                       ObjectClass::Pedestrian, 0.8};
    Detection mono;
    FusionConfig fc;
    CHECK(mono_ground_position(cam, cd, fc, mono, 0.0));
    CHECK_NEAR(mono.pos.x(), 21.5 + 0.3, 0.2);   // + half pedestrian length
    CHECK(mono.cov(0, 0) > mono.cov(1, 1));      // range less certain than bearing

    // Fusion: LiDAR car + matching YOLO box -> class "car"; debris stays unknown.
    const auto cloud = synthetic_cloud();
    const auto obs = detect_obstacles(cloud, LidarConfig{});
    BBox2D car_box{kInf, kInf, -kInf, -kInf};
    for (int idx : obs[0].point_indices) {
        Eigen::Vector2d p;
        if (cam.project({cloud[idx].x, cloud[idx].y, cloud[idx].z}, p)) {
            car_box.x1 = std::min(car_box.x1, p.x()); car_box.y1 = std::min(car_box.y1, p.y());
            car_box.x2 = std::max(car_box.x2, p.x()); car_box.y2 = std::max(car_box.y2, p.y());
        }
    }
    const auto fused = fuse_camera_lidar(cam, {{car_box, ObjectClass::Car, 0.9}}, obs, cloud, fc, 0.0);
    CHECK(fused.size() == 2);
    if (fused.size() == 2) {
        CHECK(fused[0].cls == ObjectClass::Car && fused[0].source == Sensor::Fused);
        CHECK(fused[1].cls == ObjectClass::Unknown && fused[1].source == Sensor::Lidar);
    }
}

static void test_vehicle_to_world() {
    EgoState ego;
    ego.x = 100;
    ego.y = 50;
    ego.yaw = kPi / 2;  // facing +y
    Detection d;
    d.pos = {10, 0};    // 10 m ahead
    const Detection w = vehicle_to_world(d, ego);
    CHECK_NEAR(w.pos.x(), 100.0, 1e-9);
    CHECK_NEAR(w.pos.y(), 60.0, 1e-9);
}

// ------------------------------------------------------------------ radar
// Radar returns from vehicle-frame points moving with world velocity v (ego at speed ve).
static std::vector<RadarPoint> radar_returns(const std::vector<Eigen::Vector3d>& pts, const Eigen::Vector2d& v,
                                             double ve, const RadarMount& m) {
    std::vector<RadarPoint> out;
    const Eigen::Vector2d vs = radar_sensor_velocity(m, ve, 0.0);
    for (const auto& q : pts) {
        const Eigen::Vector3d d = q - Eigen::Vector3d(m.x, m.y, m.z);
        const double r = d.norm();
        out.push_back({float(r), float(std::atan2(d.y(), d.x())), float(std::asin(d.z() / r)),
                       float((v - vs).dot(d.head<2>() / r)), 0.0f});
    }
    return out;
}

static std::vector<Eigen::Vector3d> face(double x, double y, double w, double z0, double z1, int n) {
    std::vector<Eigen::Vector3d> p;
    for (int i = 0; i < n; ++i)
        p.push_back({x, y - w / 2 + w * (i + 0.5) / n, z0 + (z1 - z0) * ((i * 7) % n + 0.5) / n});
    return p;
}

static void test_radar_doppler_compensation() {
    const RadarMount m;
    const RadarConfig cfg;
    // Stationary truck rear 60 m ahead and slightly left, ego at 25 m/s: static after compensation
    auto obs = process_radar(radar_returns(face(60, 1.0, 2.5, 0.4, 3.4, 8), {0, 0}, 25.0, m), m, cfg, 25.0, 0.0);
    CHECK(obs.size() == 1);
    if (!obs.empty()) {
        CHECK(obs[0].is_static);
        CHECK_NEAR(obs[0].v_comp, 0.0, 0.05);
        CHECK(obs[0].range_rate < -24.0);  // raw Doppler: closing at ~ego speed
    }
    // Lead car 40 m ahead moving at 15 m/s: compensated Doppler = its own speed along the LOS
    obs = process_radar(radar_returns(face(40, 0.0, 1.9, 0.4, 1.4, 6), {15, 0}, 25.0, m), m, cfg, 25.0, 0.0);
    CHECK(obs.size() == 1 && !obs[0].is_static);
    if (!obs.empty()) CHECK_NEAR(obs[0].v_comp, 15.0, 0.1);
    // Multipath ghost below the road is dropped
    obs = process_radar(radar_returns(face(40, 0.0, 1.9, -1.5, -1.0, 6), {0, 0}, 25.0, m), m, cfg, 25.0, 0.0);
    CHECK(obs.empty());
}

static void test_radar_elevation_gating() {
    const RadarMount m;
    const RadarConfig cfg;
    auto one = [&](double x, double z0, double z1) {
        auto o = process_radar(radar_returns(face(x, 0.0, 6.0, z0, z1, 10), {0, 0}, 25.0, m), m, cfg, 25.0, 0.0);
        return o.empty() ? RadarObject{} : o[0];
    };
    const RadarObject truck = one(80, 0.4, 3.4), gantry = one(80, 5.5, 6.5), plate = one(30, 0.0, 0.05);
    CHECK(truck.height_confirmed && !truck.rejected);   // stopped truck: real obstacle
    CHECK(gantry.rejected);                             // overhead sign: ignored
    CHECK(plate.rejected && !plate.height_confirmed);   // manhole cover: ignored
}

static void test_ekf_doppler_speeds_up_velocity() {
    // Stationary object 80 m ahead, ego at 20 m/s: compare velocity uncertainty after 4 frames
    // with position-only updates vs position + Doppler.
    auto run = [](bool doppler) {
        KalmanCA kf({80.0, 0.0}, Eigen::Matrix2d::Identity() * 0.25, 15.0, 3.0, 2.5);
        for (int k = 1; k <= 4; ++k) {
            const double t = 0.05 * k;
            kf.predict(0.05);
            kf.update({80.0, 0.0}, Eigen::Matrix2d::Identity() * 0.25);
            if (doppler) kf.update_range_rate(0.0 - 20.0, 0.01, {20.0 * t, 0.0}, {20.0, 0.0});
        }
        return std::make_pair(kf.vel().x(), std::sqrt(kf.cov()(2, 2)));
    };
    const auto pos_only = run(false), with_doppler = run(true);
    CHECK(with_doppler.second < 0.3 * pos_only.second);
    CHECK_NEAR(with_doppler.first, 0.0, 0.3);
}

static void test_radar_fusion() {
    Detection lidar_truck;
    lidar_truck.pos = {50.0, 0.0};
    lidar_truck.length = 9.0;
    lidar_truck.cls = ObjectClass::Truck;
    auto radar_det = [](double x, double y) {
        Detection d;
        d.pos = {x, y};
        d.source = Sensor::Radar;
        d.has_range_rate = true;
        d.range_rate = -25.0;
        d.cls = ObjectClass::Unknown;
        return d;
    };
    // Rear-face radar cluster matches the LiDAR truck (centre is 4.5 m behind the face);
    // a second cluster of the same truck is absorbed; a far object is kept as radar-only.
    const auto out = fuse_radar({lidar_truck}, {radar_det(45.6, 0.2), radar_det(48.0, -0.8), radar_det(150.0, 0.0)});
    CHECK(out.size() == 2);
    if (out.size() == 2) {
        CHECK(out[0].has_range_rate && out[0].cls == ObjectClass::Truck);
        CHECK_NEAR(out[0].pos.x(), 50.0, 1e-9);  // LiDAR position kept
        CHECK(out[1].source == Sensor::Radar && out[1].pos.x() > 100.0);
    }
}

static void test_tracker_merges_duplicates() {
    // A precise LiDAR detection plus a noisy, unabsorbed radar cluster of the same truck
    // (1.2 m away, radar-size uncertainty) must end up as ONE confirmed track...
    MultiObjectTracker trk;
    for (int k = 0; k < 30; ++k) {
        const double t = 0.05 * k;
        Detection a, b;
        a.t = b.t = t;
        a.pos = {60.0, 0.0};
        b.pos = {61.2, 0.3};
        b.cov = Eigen::Matrix2d::Identity() * 2.25;  // radar-only at 60 m
        a.cls = b.cls = ObjectClass::Unknown;
        trk.update(t, {a, b});
    }
    CHECK(trk.confirmed_tracks().size() == 1);
    // ...while two precise objects 1.2 m apart (two pedestrians side by side) stay separate.
    MultiObjectTracker trk2;
    for (int k = 0; k < 30; ++k) {
        Detection a, b;
        a.t = b.t = 0.05 * k;
        a.pos = {20.0, 0.0};
        b.pos = {20.0, 1.2};
        a.cls = b.cls = ObjectClass::Pedestrian;
        trk2.update(a.t, {a, b});
    }
    CHECK(trk2.confirmed_tracks().size() == 2);
}

// -------------------------------------------------------------- scenarios
static void test_scenarios() {
    for (const Scenario& sc : standard_scenarios()) {
        const ScenarioResult r = run_scenario(sc);
        std::printf("    %-30s %s", r.name.c_str(), r.passed ? "PASS" : "FAIL");
        if (r.expect_collision)
            std::printf("  (lead time %.2f s, required %.2f s)\n", r.lead_time, r.min_lead_time);
        else
            std::printf("  (%d frames at WARNING+)\n", r.frames_at_warning_or_above);
        CHECK(r.passed);
    }
}

int main() {
    const std::vector<std::pair<const char*, std::function<void()>>> tests = {
        {"hungarian assignment", test_hungarian},
        {"kalman filter converges", test_kalman_converges},
        {"tracker keeps ids", test_tracker_keeps_ids},
        {"tracker rejects clutter", test_tracker_rejects_clutter},
        {"prediction stops braking vehicle", test_prediction_stops_braking_vehicle},
        {"ttc formula", test_ttc_formula},
        {"footprint overlap", test_footprint_overlap},
        {"braking rollout and warning boundary", test_braking_rollout_matches_stopping_distance},
        {"warning hysteresis", test_warning_hysteresis},
        {"lidar ground + clustering", test_lidar_ground_and_clusters},
        {"camera projection + fusion", test_camera_projection_and_fusion},
        {"vehicle -> world transform", test_vehicle_to_world},
        {"radar doppler compensation + ghosts", test_radar_doppler_compensation},
        {"radar elevation gating (truck/gantry/plate)", test_radar_elevation_gating},
        {"EKF doppler update", test_ekf_doppler_speeds_up_velocity},
        {"radar fusion (match/absorb/radar-only)", test_radar_fusion},
        {"tracker merges duplicates", test_tracker_merges_duplicates},
        {"synthetic scenarios (acceptance)", test_scenarios},
    };
    for (const auto& [name, fn] : tests) {
        const int before = g_failures;
        std::printf("[ RUN  ] %s\n", name);
        fn();
        std::printf("[ %s ] %s\n", g_failures == before ? " OK " : "FAIL", name);
    }
    std::printf("\n%s: %d check(s) failed\n", g_failures == 0 ? "ALL PASSED" : "FAILURES", g_failures);
    return g_failures == 0 ? 0 : 1;
}
