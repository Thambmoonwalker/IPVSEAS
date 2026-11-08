// Class-agnostic LiDAR obstacle detection (no deep learning, no PCL dependency):
//   1. region-of-interest crop
//   2. RANSAC ground-plane fit (normal constrained to be near vertical)
//   3. remove ground and overhead points
//   4. BEV occupancy grid + connected-component clustering
//   5. oriented box fit per cluster (L-shape fitting, closeness criterion)
// Every cluster is an obstacle, even if no camera class exists for it (long tail).
// Points are in the VEHICLE frame (x forward, y left, z up).
#pragma once

#include <vector>

#include "common/types.hpp"

namespace ipvseas {

struct LidarPoint {
    float x, y, z, intensity;
};

struct LidarConfig {
    double roi_x_min = -5.0, roi_x_max = 100.0;
    double roi_y_abs = 25.0;
    int ransac_iterations = 120;
    double ground_dist = 0.20;          // [m] inlier distance to the ground plane
    double max_ground_tilt_deg = 15.0;  // reject planes steeper than this
    double min_height = 0.25;           // [m] above ground: lower points are ignored
    double max_height = 3.5;            // [m] above ground: higher points (bridges, signs) ignored
    double cell_size = 0.30;            // [m] BEV grid resolution
    int min_points = 8;                 // per cluster
    double max_cluster_extent = 20.0;   // [m] larger clusters are walls / buildings
    unsigned seed = 7;
};

struct GroundPlane {
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();  // unit, pointing up
    double d = 0.0;                                      // n.p + d = 0
    int inliers = 0;
    double height_of(const Eigen::Vector3d& p) const { return normal.dot(p) + d; }
};

struct LidarObstacle {
    Eigen::Vector3d center = Eigen::Vector3d::Zero();  // vehicle frame, box centre
    double length = 0.0, width = 0.0, height = 0.0, yaw = 0.0;
    int num_points = 0;
    std::vector<int> point_indices;  // into the input cloud (used for camera fusion)
};

GroundPlane fit_ground_ransac(const std::vector<LidarPoint>& pts, const LidarConfig& cfg);

std::vector<LidarObstacle> detect_obstacles(const std::vector<LidarPoint>& pts,
                                            const LidarConfig& cfg, GroundPlane* ground_out = nullptr);

// Vehicle-frame detection with range-dependent position uncertainty.
Detection obstacle_to_detection(const LidarObstacle& o, double t);

}  // namespace ipvseas
