// Short-term trajectory prediction for tracked objects and for the ego vehicle,
// plus simple behaviour (intent) cues.
#pragma once

#include <vector>

#include "common/types.hpp"
#include "tracking/kalman_ca.hpp"
#include "tracking/tracker.hpp"

namespace ipvseas {

struct PredictionConfig {
    double horizon = 5.0;           // [s]
    double step = 0.1;              // [s]
    double hard_brake_accel = -3.0; // [m/s^2] longitudinal decel that flags "sudden braking"
    double cut_in_lat_speed = 0.5;  // [m/s] lateral speed toward ego lane that flags "cut-in"
    // Noisy filtered accelerations must not be extrapolated for seconds:
    double accel_threshold = 1.0;   // [m/s^2] longitudinal accel below this is treated as 0
    double stationary_speed = 0.5;  // [m/s] slower objects are predicted as stationary
    double significance_chi2 = 9.21;  // velocity must differ from 0 at 99 % (2 dof) to count
    double lateral_sigma_gate = 3.0;  // vehicle lateral velocity kept only if > 3 sigma
    double vru_lateral_sigma_gate = 2.0;  // pedestrians / animals: > 2 sigma (more cautious)
};

struct PredictedPose {
    double t = 0.0;                                       // time offset from now [s]
    Eigen::Vector2d pos = Eigen::Vector2d::Zero();
    Eigen::Vector2d vel = Eigen::Vector2d::Zero();
    Eigen::Matrix2d pos_cov = Eigen::Matrix2d::Zero();
    double yaw = 0.0;
};

using Trajectory = std::vector<PredictedPose>;

// Deterministic rollout of a filtered CA state:
//   * only the LONGITUDINAL acceleration (along the velocity) is extrapolated, and only if
//     it exceeds accel_threshold; lateral motion is constant-velocity
//   * objects slower than stationary_speed are predicted as stationary
//   * vehicles and animals never reverse: braking ends in a stop
//   * pedestrians and cyclists use constant velocity
Trajectory rollout_state(const Vec6& x0, ObjectClass cls, const PredictionConfig& cfg,
                         double heading_fallback);

// Mean-state conditioning: motion that is not statistically significant given the track
// covariance is removed (stationary objects stay put; vehicle lateral noise is not
// extrapolated). `road_heading` approximates the lane direction (ego heading for now).
Vec6 significant_motion(const Vec6& x, const Mat6& P, ObjectClass cls, double road_heading,
                        const PredictionConfig& cfg);

// Rollout with covariance propagation (mean + uncertainty).
Trajectory predict_track(const Track& tr, const PredictionConfig& cfg, double heading_fallback);

// Ego vehicle: constant turn rate and velocity (CTRV) with constant longitudinal accel.
Trajectory predict_ego(const EgoState& ego, const PredictionConfig& cfg);

// Ego vehicle that keeps its current motion for `delay` seconds, then brakes at `decel`
// (used to ask "can a driver / the AEB still avoid this collision?").
Trajectory predict_ego_braking(const EgoState& ego, const PredictionConfig& cfg, double delay,
                               double decel);

struct BehaviourCues {
    bool hard_braking = false;
    bool cutting_in = false;      // lateral motion into the ego lane
    bool crossing_path = false;   // VRU / animal moving toward the ego path
    double longitudinal_accel = 0.0;
    double lateral_speed_to_ego = 0.0;  // >0 means moving toward the ego centreline
};

BehaviourCues behaviour_cues(const Track& tr, const EgoState& ego, const PredictionConfig& cfg);

}  // namespace ipvseas
