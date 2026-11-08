// Multi-object tracker: CA Kalman filters + Mahalanobis gating + Hungarian association
// + M-of-N track management. Runs in the WORLD frame.
#pragma once

#include <map>
#include <vector>

#include "common/types.hpp"
#include "tracking/kalman_ca.hpp"

namespace ipvseas {

struct TrackerConfig {
    double gate_chi2 = 13.8;        // chi-square, 2 dof, 99.9 %
    int confirm_hits = 3;           // hits needed before a track is confirmed
    int max_misses = 10;            // consecutive misses before deletion (0.5 s at 20 Hz)
    double mature_vel_sigma = 1.5;  // [m/s] velocity std below which a track is "mature"
    double mature_vel_sigma_tangential = 3.0;  // [m/s] sideways limit for radar-tracked objects
    int max_coast_for_warning = 2;
    double merge_chi2 = 4.0;        // tracks closer than this (Mahalanobis^2) are duplicates  // a track not measured for longer can only raise CAUTION
};

struct Track {
    int id = -1;
    KalmanCA kf;
    ObjectClass cls = ObjectClass::Unknown;
    std::map<ObjectClass, double> class_votes;
    int hits = 0;
    int misses = 0;
    bool confirmed = false;
    bool velocity_initialised = false;
    double first_t = 0.0, last_t = 0.0;
    Eigen::Vector2d last_meas = Eigen::Vector2d::Zero();
    double length = kNaN, width = kNaN, yaw = kNaN;
    bool height_ok = false;      // confirmed solid at road level by at least one detection
    int radar_updates = 0;       // number of Doppler updates received
    Eigen::Vector2d last_sensor_pos = Eigen::Vector2d::Zero();  // radar position at last Doppler update
    int radar_height_votes = 0;    // radar-only detections that confirmed road-level height
    int solid_votes = 0;           // LiDAR / camera detections (each also confirms height)
    int doppler_static_count = 0;  // consecutive Doppler updates saying "stationary in the world"

    double vel_sigma() const;
    double vel_sigma_radial() const;      // along the radar line of sight
    double vel_sigma_tangential() const;  // across it
    // Radar repeatedly measured zero world velocity along the line of sight, the sideways
    // velocity is not significant, and it is not a pedestrian/animal (those cross with zero
    // Doppler): treat as a stationary obstacle (poor man's IMM "stationary" model).
    bool doppler_static() const;
    Eigen::Vector2d footprint() const;  // (length, width)
    double heading(double fallback) const;
};

class MultiObjectTracker {
public:
    explicit MultiObjectTracker(TrackerConfig cfg = {}) : cfg_(cfg) {}

    // Advance all tracks to time t and fuse this frame's detections.
    void update(double t, const std::vector<Detection>& detections);

    const std::vector<Track>& tracks() const { return tracks_; }
    std::vector<const Track*> confirmed_tracks() const;
    bool is_mature(const Track& tr) const;
    const TrackerConfig& config() const { return cfg_; }

    // Class-specific motion parameters.
    static double sigma_jerk_for(ObjectClass c);
    static double init_vel_sigma_for(ObjectClass c);

private:
    void start_track(const Detection& d);
    static bool classes_compatible(ObjectClass a, ObjectClass b);

    TrackerConfig cfg_;
    std::vector<Track> tracks_;
    int next_id_ = 1;
    double last_t_ = -1.0;
};

}  // namespace ipvseas
