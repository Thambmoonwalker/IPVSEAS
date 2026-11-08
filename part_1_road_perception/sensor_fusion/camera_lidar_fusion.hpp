// Late camera-LiDAR fusion:
//   * project each LiDAR obstacle's points into the image -> 2D box
//   * associate with YOLO boxes by IoU (Hungarian)
//   * matched   -> LiDAR position + camera class            (best of both)
//   * LiDAR only -> "unknown_obstacle" (kept: long-tail hazard)
//   * camera only -> monocular ground-plane range, large uncertainty
// All outputs are in the VEHICLE frame; convert with vehicle_to_world().
#pragma once

#include <vector>

#include "common/types.hpp"
#include "lidar/obstacle_detector.hpp"

namespace ipvseas {

struct CameraModel {
    double fx = 0, fy = 0, cx = 0, cy = 0;
    int width = 0, height = 0;
    // Rigid transform vehicle frame -> camera OPTICAL frame (x right, y down, z forward).
    Eigen::Matrix3d R_cv = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_cv = Eigen::Vector3d::Zero();

    // Pinhole camera with horizontal field of view (as configured in CARLA), mounted at
    // (x, y, z) in the vehicle frame with a pitch angle (positive = nose down).
    static CameraModel from_fov(int width, int height, double fov_deg, const Eigen::Vector3d& mount,
                                double pitch_rad = 0.0);

    Eigen::Vector3d to_camera(const Eigen::Vector3d& p_vehicle) const { return R_cv * p_vehicle + t_cv; }
    bool project(const Eigen::Vector3d& p_vehicle, Eigen::Vector2d& uv) const;
    Eigen::Vector3d center_vehicle() const { return -R_cv.transpose() * t_cv; }
};

struct BBox2D {
    double x1 = 0, y1 = 0, x2 = 0, y2 = 0;
    double area() const { return std::max(0.0, x2 - x1) * std::max(0.0, y2 - y1); }
};

double iou(const BBox2D& a, const BBox2D& b);

struct CameraDetection {
    BBox2D box;
    ObjectClass cls = ObjectClass::Unknown;
    double confidence = 0.0;
};

struct FusionConfig {
    double min_iou = 0.3;
    double pixel_sigma = 3.0;        // [px] bottom-edge localisation noise for mono range
    double max_mono_range = 60.0;    // [m] ignore camera-only objects beyond this
    double min_camera_conf = 0.35;
};

// Monocular range from where the box bottom touches a flat road (z = 0 in vehicle frame).
// Returns false if the ray does not hit the ground in front of the camera.
bool mono_ground_position(const CameraModel& cam, const CameraDetection& det,
                          const FusionConfig& cfg, Detection& out, double t);

std::vector<Detection> fuse_camera_lidar(const CameraModel& cam,
                                         const std::vector<CameraDetection>& cam_dets,
                                         const std::vector<LidarObstacle>& lidar_obs,
                                         const std::vector<LidarPoint>& cloud,
                                         const FusionConfig& cfg, double t);

}  // namespace ipvseas
