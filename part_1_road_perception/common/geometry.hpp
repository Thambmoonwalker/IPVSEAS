// Frame conversions between the ego vehicle frame and the world frame.
#pragma once

#include "common/types.hpp"

namespace ipvseas {

inline Eigen::Matrix2d rot2(double yaw) {
    Eigen::Matrix2d R;
    R << std::cos(yaw), -std::sin(yaw), std::sin(yaw), std::cos(yaw);
    return R;
}

// Detection measured in the vehicle frame -> world frame using the ego pose.
inline Detection vehicle_to_world(Detection d, const EgoState& ego) {
    const Eigen::Matrix2d R = rot2(ego.yaw);
    d.pos = Eigen::Vector2d(ego.x, ego.y) + R * d.pos;
    d.cov = R * d.cov * R.transpose();
    if (std::isfinite(d.yaw)) d.yaw = wrap_angle(d.yaw + ego.yaw);
    if (d.has_range_rate) {
        // sensor_vel was the sensor's velocity in the vehicle frame (rigid-body motion).
        d.sensor_pos = Eigen::Vector2d(ego.x, ego.y) + R * d.sensor_pos;
        d.sensor_vel = R * d.sensor_vel;
    }
    return d;
}

inline Eigen::Vector2d world_to_vehicle(const Eigen::Vector2d& p, const EgoState& ego) {
    return rot2(ego.yaw).transpose() * (p - Eigen::Vector2d(ego.x, ego.y));
}

}  // namespace ipvseas
