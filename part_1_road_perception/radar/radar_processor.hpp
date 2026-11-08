// 4D imaging radar processing: each detection gives range, azimuth, elevation and radial
// (Doppler) velocity. Pipeline:
//   1. polar -> Cartesian in the vehicle frame (mount position and yaw applied)
//   2. ego-motion compensation of the Doppler: static world points -> v_comp ~ 0
//   3. ghost filtering (points below the road surface = multipath reflections)
//   4. DBSCAN clustering in position + compensated velocity (moving and static kept apart)
//   5. per cluster: centroid, mean Doppler, height statistics, micro-Doppler spread
//   6. elevation gating of STATIONARY clusters: keep road-level objects (stopped truck),
//      reject overhead structures (bridges, gantries); "unconfirmed" if too uncertain
// Output: vehicle-frame Detections carrying a range-rate measurement for the tracker's EKF.
#pragma once

#include <vector>

#include "common/types.hpp"

namespace ipvseas {

// One raw radar detection in the SENSOR frame (ISO: azimuth positive to the LEFT).
struct RadarPoint {
    float range;      // [m]
    float azimuth;    // [rad]
    float elevation;  // [rad], positive up
    float range_rate; // [m/s], relative to the sensor, positive = moving away
    float rcs;        // reflection strength (dBsm or SNR); 0 if unknown
};

struct RadarMount {
    double x = 2.3, y = 0.0, z = 0.5;   // front bumper (vehicle frame origin = vehicle centre) [m]
    double yaw = 0.0;                   // [rad]
};

struct RadarConfig {
    double max_range = 200.0;           // long-range front radar mode
    double half_fov = 0.79;             // [rad] +-45 deg
    // Near field: an extended object spans a wide angle, so the cluster's mean Doppler no
    // longer equals the Doppler at its centroid. Skip Doppler there and near the FOV edge.
    double doppler_min_range = 10.0;    // [m]
    double doppler_fov_margin = 0.17;   // [rad] ~10 deg
    double sigma_range = 0.15;          // [m]
    double sigma_azimuth = 0.0087;      // [rad]  ~0.5 deg (imaging-radar class)
    double sigma_elevation = 0.0087;    // [rad]  ~0.5 deg
    double sigma_range_rate = 0.10;     // [m/s]
    double static_velocity = 0.6;       // [m/s] |v_comp| below this = static world point
    double ghost_below_ground = -0.5;   // [m] points lower than this are multipath ghosts
    double cluster_eps = 2.0;           // [m] DBSCAN neighbourhood (radar is sparse); grows with
                                        // range as 2.5 x range x sigma_azimuth
    double cluster_eps_velocity = 1.0;  // [m/s] points must also agree in Doppler
    int cluster_min_points = 2;
    // Height band (above the road, z = 0) for a solid road obstacle
    double obstacle_min_height = 0.3, obstacle_max_height = 4.0;
};

struct RadarObject {
    Eigen::Vector2d pos = Eigen::Vector2d::Zero();  // vehicle frame centroid [m]
    double range = 0.0, azimuth = 0.0;
    double z_mean = 0.0, z_sigma = 0.0;              // height estimate and its std (of the mean)
    double range_rate = 0.0;                         // mean raw Doppler (relative to sensor)
    double v_comp = 0.0;                             // mean ego-compensated Doppler
    double doppler_spread = 0.0;                     // std of v_comp inside the cluster (micro-Doppler)
    double range_rate_spread = 0.0;                  // std of raw Doppler inside the cluster
    int num_points = 0;
    bool is_static = false;
    bool height_confirmed = true;   // static objects: height certainly inside the obstacle band
    bool rejected = false;          // static object certainly outside the band (bridge, plate)
};

// Sensor velocity in the vehicle frame for ego speed v and yaw rate w (rigid body).
Eigen::Vector2d radar_sensor_velocity(const RadarMount& m, double ego_speed, double ego_yaw_rate);

// Steps 1-6. Rejected objects are returned too (flagged) so they can be logged / visualised.
std::vector<RadarObject> process_radar(const std::vector<RadarPoint>& pts, const RadarMount& mount,
                                       const RadarConfig& cfg, double ego_speed, double ego_yaw_rate);

// Usable for tracking: not rejected by elevation gating, and not cut off by the edge of the
// field of view (a truncated cluster's centroid slides toward the FOV centre).
bool radar_object_usable(const RadarObject& o, const RadarMount& mount, const RadarConfig& cfg);

// Vehicle-frame Detection with range-rate measurement (call only for usable objects).
Detection radar_object_to_detection(const RadarObject& o, const RadarMount& mount,
                                    const RadarConfig& cfg, double ego_speed, double ego_yaw_rate,
                                    double t);

}  // namespace ipvseas
