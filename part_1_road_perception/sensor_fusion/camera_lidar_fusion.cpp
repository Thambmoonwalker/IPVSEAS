#include "sensor_fusion/camera_lidar_fusion.hpp"

#include <algorithm>

#include "tracking/hungarian.hpp"

namespace ipvseas {

CameraModel CameraModel::from_fov(int width, int height, double fov_deg, const Eigen::Vector3d& mount,
                                  double pitch_rad) {
    CameraModel c;
    c.width = width;
    c.height = height;
    c.fx = c.fy = width / (2.0 * std::tan(fov_deg * kPi / 360.0));
    c.cx = width / 2.0;
    c.cy = height / 2.0;
    // Vehicle axes -> optical axes: x_opt = -y_v, y_opt = -z_v, z_opt = x_v.
    Eigen::Matrix3d R_opt;
    R_opt << 0, -1, 0,
             0, 0, -1,
             1, 0, 0;
    // Mount pitch (nose down positive) about the vehicle y axis.
    const Eigen::Matrix3d R_pitch = Eigen::AngleAxisd(pitch_rad, Eigen::Vector3d::UnitY()).toRotationMatrix();
    c.R_cv = R_opt * R_pitch.transpose();
    c.t_cv = -c.R_cv * mount;
    return c;
}

bool CameraModel::project(const Eigen::Vector3d& p_vehicle, Eigen::Vector2d& uv) const {
    const Eigen::Vector3d pc = to_camera(p_vehicle);
    if (pc.z() < 0.5) return false;  // behind or too close to the lens
    uv = {fx * pc.x() / pc.z() + cx, fy * pc.y() / pc.z() + cy};
    return true;
}

double iou(const BBox2D& a, const BBox2D& b) {
    BBox2D i{std::max(a.x1, b.x1), std::max(a.y1, b.y1), std::min(a.x2, b.x2), std::min(a.y2, b.y2)};
    const double inter = i.area();
    const double uni = a.area() + b.area() - inter;
    return uni > 0 ? inter / uni : 0.0;
}

bool mono_ground_position(const CameraModel& cam, const CameraDetection& det,
                          const FusionConfig& cfg, Detection& out, double t) {
    const double u = 0.5 * (det.box.x1 + det.box.x2), v = det.box.y2;
    const Eigen::Vector3d ray_cam((u - cam.cx) / cam.fx, (v - cam.cy) / cam.fy, 1.0);
    const Eigen::Vector3d ray_v = cam.R_cv.transpose() * ray_cam;
    const Eigen::Vector3d c = cam.center_vehicle();
    if (ray_v.z() >= -1e-6) return false;  // ray points at or above the horizon
    const double s = -c.z() / ray_v.z();
    const Eigen::Vector3d g = c + s * ray_v;
    const double range = g.head<2>().norm();
    if (g.x() <= 0.0 || range > cfg.max_mono_range) return false;

    // Range error from bottom-edge pixel error: dZ = Z^2 / (f * h) * dv
    const double h = c.z();
    const double sig_r = range * range / (cam.fy * h) * cfg.pixel_sigma + 0.3;
    const double sig_l = range * cfg.pixel_sigma / cam.fx + 0.2;
    const Eigen::Vector2d dir = g.head<2>() / range, perp(-dir.y(), dir.x());
    out = Detection{};
    out.t = t;
    out.pos = g.head<2>();
    // Shift from the visible near edge to the object centre (half the default length).
    out.pos += dir * 0.5 * default_footprint(det.cls).x();
    out.cov = sig_r * sig_r * dir * dir.transpose() + sig_l * sig_l * perp * perp.transpose();
    out.cls = det.cls;
    out.confidence = det.confidence;
    out.source = Sensor::Camera;
    return true;
}

std::vector<Detection> fuse_camera_lidar(const CameraModel& cam,
                                         const std::vector<CameraDetection>& cam_dets_in,
                                         const std::vector<LidarObstacle>& lidar_obs,
                                         const std::vector<LidarPoint>& cloud,
                                         const FusionConfig& cfg, double t) {
    std::vector<CameraDetection> cam_dets;
    for (const auto& d : cam_dets_in)
        if (d.confidence >= cfg.min_camera_conf) cam_dets.push_back(d);

    // 2D box of each LiDAR obstacle's projected points (empty if outside the image).
    std::vector<BBox2D> lidar_boxes(lidar_obs.size());
    std::vector<bool> visible(lidar_obs.size(), false);
    for (size_t k = 0; k < lidar_obs.size(); ++k) {
        BBox2D b{kInf, kInf, -kInf, -kInf};
        int n = 0;
        for (int idx : lidar_obs[k].point_indices) {
            Eigen::Vector2d uv;
            if (!cam.project({cloud[idx].x, cloud[idx].y, cloud[idx].z}, uv)) continue;
            if (uv.x() < 0 || uv.y() < 0 || uv.x() >= cam.width || uv.y() >= cam.height) continue;
            b.x1 = std::min(b.x1, uv.x()); b.y1 = std::min(b.y1, uv.y());
            b.x2 = std::max(b.x2, uv.x()); b.y2 = std::max(b.y2, uv.y());
            ++n;
        }
        if (n >= 3) { lidar_boxes[k] = b; visible[k] = true; }
    }

    // Cost = 1 - IoU, forbidden below min_iou.
    const double forbidden = 1.0 - cfg.min_iou;
    Eigen::MatrixXd cost = Eigen::MatrixXd::Constant(lidar_obs.size(), cam_dets.size(), 1.0);
    for (size_t k = 0; k < lidar_obs.size(); ++k) {
        if (!visible[k]) continue;
        for (size_t j = 0; j < cam_dets.size(); ++j) {
            const double c = 1.0 - iou(lidar_boxes[k], cam_dets[j].box);
            if (c < forbidden) cost(k, j) = c;
        }
    }
    const std::vector<int> assign = hungarian_assign(cost, forbidden);

    std::vector<Detection> out;
    std::vector<bool> cam_used(cam_dets.size(), false);
    for (size_t k = 0; k < lidar_obs.size(); ++k) {
        Detection d = obstacle_to_detection(lidar_obs[k], t);
        if (assign[k] >= 0) {
            const CameraDetection& cd = cam_dets[assign[k]];
            cam_used[assign[k]] = true;
            d.cls = cd.cls;
            d.confidence = std::max(d.confidence, cd.confidence);
            d.source = Sensor::Fused;
        }
        out.push_back(d);
    }
    for (size_t j = 0; j < cam_dets.size(); ++j) {
        if (cam_used[j]) continue;
        Detection d;
        if (mono_ground_position(cam, cam_dets[j], cfg, d, t)) out.push_back(d);
    }
    return out;
}

}  // namespace ipvseas
