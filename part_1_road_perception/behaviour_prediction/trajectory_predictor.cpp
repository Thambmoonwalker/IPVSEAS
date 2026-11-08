#include "behaviour_prediction/trajectory_predictor.hpp"

namespace ipvseas {

namespace {

bool uses_constant_velocity(ObjectClass cls) {
    return group_of(cls) == ClassGroup::VulnerableRoadUser;
}

// Keep only a significant longitudinal acceleration; zero everything else.
Vec6 condition_state(Vec6 x, ObjectClass cls, const PredictionConfig& cfg) {
    const Eigen::Vector2d v = x.segment<2>(2);
    if (v.norm() < cfg.stationary_speed) {
        x.segment<4>(2).setZero();
        return x;
    }
    if (uses_constant_velocity(cls)) {
        x.tail<2>().setZero();
        return x;
    }
    const Eigen::Vector2d u = v.normalized();
    const double a_long = x.tail<2>().dot(u);
    x.tail<2>() = std::abs(a_long) > cfg.accel_threshold ? Eigen::Vector2d(a_long * u)
                                                         : Eigen::Vector2d::Zero();
    return x;
}

// One deterministic step; `stopped` latches once a braking object comes to rest.
void step_state(Vec6& x, double dt, bool& stopped) {
    if (stopped) return;
    const Eigen::Vector2d v = x.segment<2>(2);
    const Eigen::Vector2d a = x.tail<2>();
    Eigen::Vector2d v_next = v + a * dt;
    if (v.norm() > 1e-3 && v_next.dot(v) <= 0.0) {
        // Braking to a stop inside this step: travel to the stopping point, then halt.
        const double along = -a.dot(v) / v.norm();  // deceleration magnitude along v
        const double t_stop = along > 1e-6 ? v.norm() / along : dt;
        x.head<2>() += v * t_stop * 0.5;
        x.segment<2>(2).setZero();
        x.tail<2>().setZero();
        stopped = true;
        return;
    }
    x.head<2>() += v * dt + 0.5 * a * dt * dt;
    x.segment<2>(2) = v_next;
}

}  // namespace

Trajectory rollout_state(const Vec6& x0, ObjectClass cls, const PredictionConfig& cfg,
                         double heading_fallback) {
    Trajectory traj;
    const int n = static_cast<int>(std::round(cfg.horizon / cfg.step));
    traj.reserve(n + 1);
    Vec6 x = condition_state(x0, cls, cfg);
    bool stopped = x.segment<2>(2).norm() == 0.0;
    double yaw = x.segment<2>(2).norm() > 1.0 ? std::atan2(x(3), x(2)) : heading_fallback;
    for (int k = 0; k <= n; ++k) {
        if (k > 0) step_state(x, cfg.step, stopped);
        if (x.segment<2>(2).norm() > 1.0) yaw = std::atan2(x(3), x(2));
        PredictedPose p;
        p.t = k * cfg.step;
        p.pos = x.head<2>();
        p.vel = x.segment<2>(2);
        p.yaw = yaw;
        traj.push_back(p);
    }
    return traj;
}

Vec6 significant_motion(const Vec6& x, const Mat6& P, ObjectClass cls, double road_heading,
                        const PredictionConfig& cfg) {
    Vec6 out = x;
    const Eigen::Vector2d v = x.segment<2>(2);
    const Eigen::Matrix2d Pv = P.block<2, 2>(2, 2);
    const Eigen::Vector2d f(std::cos(road_heading), std::sin(road_heading)), l(-f.y(), f.x());
    const double v_lat = v.dot(l);
    const double s_lat = std::sqrt(std::max(0.0, l.dot(Pv * l)));
    // Pedestrians, cyclists and animals: their slow motion is what matters, so keep it,
    // but only extrapolate sideways (toward the road) motion that is clearly real.
    if (group_of(cls) == ClassGroup::VulnerableRoadUser || group_of(cls) == ClassGroup::Animal) {
        if (v.norm() < 0.2) {
            out.segment<4>(2).setZero();
        } else if (std::abs(v_lat) < cfg.vru_lateral_sigma_gate * s_lat) {
            out.segment<2>(2) = v - v_lat * l;
        }
        return out;
    }
    // Not significantly different from standing still -> stationary.
    if (v.norm() < cfg.stationary_speed || v.dot(Pv.ldlt().solve(v)) < cfg.significance_chi2) {
        out.segment<4>(2).setZero();
        return out;
    }
    // Vehicles move along the road: keep lateral velocity only if it is significant.
    if (group_of(cls) == ClassGroup::Vehicle || cls == ObjectClass::Unknown) {
        if (std::abs(v_lat) < cfg.lateral_sigma_gate * s_lat) out.segment<2>(2) = v - v_lat * l;
    }
    return out;
}

Trajectory predict_track(const Track& tr, const PredictionConfig& cfg, double heading_fallback) {
    const Vec6 x = significant_motion(tr.kf.state(), tr.kf.cov(), tr.cls, heading_fallback, cfg);
    Trajectory traj = rollout_state(x, tr.cls, cfg, tr.heading(heading_fallback));
    // Propagate covariance with the linear CA model (uncertainty grows with horizon).
    const Mat6 F = KalmanCA::transition(cfg.step);
    const Mat6 Q = KalmanCA::process_noise(cfg.step, tr.kf.sigma_jerk());
    Mat6 P = tr.kf.cov();
    for (size_t k = 0; k < traj.size(); ++k) {
        if (k > 0) P = F * P * F.transpose() + Q;
        traj[k].pos_cov = P.topLeftCorner<2, 2>();
    }
    return traj;
}

Trajectory predict_ego(const EgoState& ego, const PredictionConfig& cfg) {
    Trajectory traj;
    const int n = static_cast<int>(std::round(cfg.horizon / cfg.step));
    traj.reserve(n + 1);
    double x = ego.x, y = ego.y, yaw = ego.yaw, v = ego.speed;
    for (int k = 0; k <= n; ++k) {
        if (k > 0) {
            const double dt = cfg.step;
            x += v * std::cos(yaw) * dt;
            y += v * std::sin(yaw) * dt;
            yaw = wrap_angle(yaw + ego.yaw_rate * dt);
            v = std::max(0.0, v + ego.accel * dt);
        }
        PredictedPose p;
        p.t = k * cfg.step;
        p.pos = {x, y};
        p.vel = {v * std::cos(yaw), v * std::sin(yaw)};
        p.yaw = yaw;
        traj.push_back(p);
    }
    return traj;
}

Trajectory predict_ego_braking(const EgoState& ego, const PredictionConfig& cfg, double delay,
                               double decel) {
    const Trajectory free = predict_ego(ego, cfg);
    Trajectory traj = free;
    // Re-integrate the longitudinal motion along the same path with the braking profile.
    double v = ego.speed, s = 0.0;
    std::vector<double> s_cv(free.size(), 0.0);  // arc length of the free-running path
    for (size_t k = 1; k < free.size(); ++k)
        s_cv[k] = s_cv[k - 1] + (free[k].pos - free[k - 1].pos).norm();
    size_t j = 0;
    for (size_t k = 0; k < traj.size(); ++k) {
        if (k > 0) {
            const double dt = cfg.step;
            const double t_start = free[k - 1].t;  // braking applies from `delay` onwards
            const double a = t_start >= delay - 1e-9 ? -decel : ego.accel;
            const double v_next = std::max(0.0, v + a * dt);
            s += 0.5 * (v + v_next) * dt;
            v = v_next;
        }
        // Position at arc length s on the free-running path (linear interpolation).
        while (j + 1 < free.size() && s_cv[j + 1] < s) ++j;
        PredictedPose p = free[j];
        if (j + 1 < free.size() && s_cv[j + 1] > s_cv[j]) {
            const double f = std::min(1.0, (s - s_cv[j]) / (s_cv[j + 1] - s_cv[j]));
            p.pos = free[j].pos + f * (free[j + 1].pos - free[j].pos);
        }
        p.t = free[k].t;
        p.vel = Eigen::Vector2d(std::cos(p.yaw), std::sin(p.yaw)) * v;
        traj[k] = p;
    }
    return traj;
}

BehaviourCues behaviour_cues(const Track& tr, const EgoState& ego, const PredictionConfig& cfg) {
    BehaviourCues c;
    const Eigen::Vector2d v = tr.kf.vel(), a = tr.kf.acc();
    if (v.norm() > 1.0) c.longitudinal_accel = a.dot(v.normalized());
    c.hard_braking = group_of(tr.cls) == ClassGroup::Vehicle && v.norm() > 2.0 &&
                     c.longitudinal_accel < cfg.hard_brake_accel;

    // Lateral position/velocity relative to the ego heading.
    const Eigen::Vector2d left(-std::sin(ego.yaw), std::cos(ego.yaw));
    const double lat = (tr.kf.pos() - Eigen::Vector2d(ego.x, ego.y)).dot(left);
    const double lat_v = v.dot(left);
    c.lateral_speed_to_ego = lat > 0 ? -lat_v : lat_v;
    const bool vehicle = group_of(tr.cls) == ClassGroup::Vehicle || tr.cls == ObjectClass::Unknown;
    c.cutting_in = vehicle && std::abs(lat) > 1.0 && c.lateral_speed_to_ego > cfg.cut_in_lat_speed;
    const bool vru = group_of(tr.cls) == ClassGroup::VulnerableRoadUser ||
                     group_of(tr.cls) == ClassGroup::Animal;
    c.crossing_path = vru && c.lateral_speed_to_ego > 0.3;
    return c;
}

}  // namespace ipvseas
