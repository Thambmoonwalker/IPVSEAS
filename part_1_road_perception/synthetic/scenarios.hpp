// Synthetic scenario simulator for Part 1.
// Generates ground truth for an "inattentive" ego driver (constant speed, no reaction),
// produces noisy world-frame detections, runs the Part 1 pipeline and measures:
//   - true collision time (if the driver never reacts)
//   - first CAUTION / WARNING / CRITICAL times and the warning lead time
//   - TTC estimation error, false-alarm frames
// This lets the algorithms be developed and regression-tested without CARLA.
#pragma once

#include <functional>
#include <string>
#include <vector>

#include "part1_pipeline.hpp"

namespace ipvseas {

struct ActorState {
    Eigen::Vector2d pos = Eigen::Vector2d::Zero();
    Eigen::Vector2d vel = Eigen::Vector2d::Zero();
};

struct SyntheticActor {
    std::string name;
    ObjectClass cls = ObjectClass::Car;
    Eigen::Vector2d size = {4.5, 1.9};  // length, width
    ActorState init;
    // Commanded acceleration [m/s^2] as a function of time and current state.
    std::function<Eigen::Vector2d(double, const ActorState&)> accel;
    // Commanded velocity override (e.g. walking pedestrians); return false to keep integrating.
    std::function<bool(double, ActorState&)> velocity_override;
    // Extra visibility rule (e.g. occlusion behind a parked car).
    std::function<bool(double, const ActorState&)> visible;
};

// Static structure seen only by radar (LiDAR's height band and the camera ignore it),
// e.g. an overhead sign gantry or a manhole cover: the classic radar false-alarm sources.
struct RadarStructure {
    std::string name;
    Eigen::Vector2d pos = Eigen::Vector2d::Zero();  // world, centre
    double length = 0.5, width = 8.0;
    double z_bottom = 5.5, z_top = 6.5;              // [m] above the road
};

struct SensorModel {
    // Primary sensor = camera + LiDAR fused (positions)
    double max_range = 100.0;     // [m]
    double min_x = 0.0;           // front-facing: only objects ahead of the ego centre
    double detect_prob = 0.95;
    double pos_sigma_base = 0.15; // [m]
    double pos_sigma_per_m = 0.003;
    double clutter_rate = 0.03;   // false detections per frame
    // 4D imaging radar (front bumper)
    bool radar = true;
    double radar_range = 200.0;
    double radar_half_fov = 0.79; // [rad] +-45 deg
    double radar_points_at_50m = 6.0;  // returns from a car at 50 m (scaled by size and range)
    double radar_point_prob = 0.9;
    int guardrail_points = 12;    // static clutter per frame along both road edges (y = +-7 m)
    double ghost_prob = 0.3;      // chance per object of a below-road multipath ghost
    unsigned seed = 1;
};

struct Scenario {
    std::string name;
    std::string description;
    double ego_speed = 25.0;
    double duration = 10.0;
    double dt = 0.05;             // 20 Hz
    std::vector<SyntheticActor> actors;
    std::vector<RadarStructure> structures;
    SensorModel sensor;
    bool expect_collision = true; // false = false-alarm test
    double min_lead_time = 1.5;   // acceptance criterion for collision scenarios [s]
};

struct ScenarioResult {
    std::string name;
    bool expect_collision = true;
    double t_collision = kInf;    // true, with no driver reaction
    double t_first_caution = kInf, t_first_warning = kInf, t_first_critical = kInf;
    double lead_time = -kInf;     // t_collision - t_first_warning
    int frames_at_warning_or_above = 0;
    WarningLevel max_level = WarningLevel::None;
    double ttc_mae = kNaN;        // mean |predicted TTC - true remaining time| (last 5 s)
    int ttc_samples = 0;
    double min_lead_time = 0.0;
    bool passed = false;
    std::string csv;              // per-frame log
};

std::vector<Scenario> standard_scenarios();
ScenarioResult run_scenario(const Scenario& sc, const Part1Config& cfg = {});

}  // namespace ipvseas
