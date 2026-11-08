#include "collision_prediction/collision_risk.hpp"

#include <algorithm>
#include <random>

namespace ipvseas {

namespace {

// Project an oriented rectangle onto an axis; return half-extent of the projection.
double half_extent_on(const Eigen::Vector2d& axis, double yaw, double len, double wid) {
    const Eigen::Vector2d f(std::cos(yaw), std::sin(yaw));
    const Eigen::Vector2d l(-std::sin(yaw), std::cos(yaw));
    return 0.5 * len * std::abs(axis.dot(f)) + 0.5 * wid * std::abs(axis.dot(l));
}

}  // namespace

bool footprints_overlap(const Eigen::Vector2d& c1, double yaw1, double len1, double wid1,
                        const Eigen::Vector2d& c2, double yaw2, double len2, double wid2) {
    const Eigen::Vector2d d = c2 - c1;
    const Eigen::Vector2d axes[4] = {
        {std::cos(yaw1), std::sin(yaw1)}, {-std::sin(yaw1), std::cos(yaw1)},
        {std::cos(yaw2), std::sin(yaw2)}, {-std::sin(yaw2), std::cos(yaw2)}};
    for (const Eigen::Vector2d& ax : axes) {
        const double r = half_extent_on(ax, yaw1, len1, wid1) + half_extent_on(ax, yaw2, len2, wid2);
        if (std::abs(d.dot(ax)) > r) return false;  // separating axis found
    }
    return true;
}

double first_conflict_time(const Trajectory& ego, const EgoState& ego_state, const Trajectory& obj,
                           const Eigen::Vector2d& obj_size, double margin) {
    const size_t n = std::min(ego.size(), obj.size());
    for (size_t k = 0; k < n; ++k) {
        if (footprints_overlap(ego[k].pos, ego[k].yaw, ego_state.length + 2 * margin,
                               ego_state.width + 2 * margin, obj[k].pos, obj[k].yaw,
                               obj_size.x() + 2 * margin, obj_size.y() + 2 * margin))
            return ego[k].t;
    }
    return kInf;
}

double time_to_collision_ca(double gap, double v_rel, double a_rel) {
    if (gap <= 0.0) return 0.0;
    if (std::abs(a_rel) < 1e-6) return v_rel < 0.0 ? gap / -v_rel : kInf;
    // 0.5 a t^2 + v t + gap = 0
    const double disc = v_rel * v_rel - 2.0 * a_rel * gap;
    if (disc < 0.0) return kInf;
    const double s = std::sqrt(disc);
    const double t1 = (-v_rel - s) / a_rel, t2 = (-v_rel + s) / a_rel;
    double best = kInf;
    for (double t : {t1, t2})
        if (t > 0.0) best = std::min(best, t);
    return best;
}

double CollisionRiskAssessor::collision_probability(const Track& tr, const EgoState& ego,
                                                    const Trajectory& ego_traj) const {
    // Sample position and velocity from the track covariance around the significance-gated
    // mean state; keep the gated mean acceleration (sampling the filter's acceleration
    // noise would make stationary objects "drive away" in many samples).
    const Vec6 mean = significant_motion(tr.kf.state(), tr.kf.cov(), tr.cls, ego.yaw, pc_);
    Eigen::Matrix4d P = tr.kf.cov().topLeftCorner<4, 4>();
    if (tr.doppler_static()) {
        // Stationary obstacle confirmed by Doppler: keep the position uncertainty, but the
        // velocity uncertainty is small (no sideways drift from long-range azimuth noise).
        P.block<2, 2>(0, 2).setZero();
        P.block<2, 2>(2, 0).setZero();
        P.block<2, 2>(2, 2) = Eigen::Matrix2d::Identity() * 0.3 * 0.3;
    }
    P.diagonal().array() += 1e-9;
    const Eigen::LLT<Eigen::Matrix4d> llt(P);
    if (llt.info() != Eigen::Success) return 0.0;
    const Eigen::Matrix4d L = llt.matrixL();
    std::mt19937 rng(rc_.seed + static_cast<unsigned>(tr.id));
    std::normal_distribution<double> N01(0.0, 1.0);
    const Eigen::Vector2d size = tr.footprint();
    const double heading = tr.heading(ego.yaw);
    int hits = 0;
    for (int s = 0; s < rc_.mc_samples; ++s) {
        Eigen::Vector4d e;
        for (int i = 0; i < 4; ++i) e(i) = N01(rng);
        Vec6 x = mean;
        x.head<4>() += L * e;
        const Trajectory traj = rollout_state(x, tr.cls, pc_, heading);
        if (std::isfinite(first_conflict_time(ego_traj, ego, traj, size, rc_.safety_margin))) ++hits;
    }
    return static_cast<double>(hits) / rc_.mc_samples;
}

CollisionRiskAssessor::EgoPlans CollisionRiskAssessor::plan_ego(const EgoState& ego) const {
    EgoPlans p;
    p.free = predict_ego(ego, pc_);
    p.caution = predict_ego_braking(ego, pc_, rc_.driver_reaction_time + rc_.caution_margin,
                                    rc_.driver_decel);
    p.driver = predict_ego_braking(ego, pc_, rc_.driver_reaction_time, rc_.driver_decel);
    p.aeb = predict_ego_braking(ego, pc_, rc_.system_latency, rc_.max_decel);
    return p;
}

Hazard CollisionRiskAssessor::assess_track(const Track& tr, const EgoState& ego,
                                           const EgoPlans& plans, bool mature) const {
    const Trajectory& ego_traj = plans.free;
    Hazard h;
    h.track_id = tr.id;
    h.cls = tr.cls;
    h.cues = behaviour_cues(tr, ego, pc_);

    const Eigen::Vector2d ego_p(ego.x, ego.y);
    const Eigen::Vector2d ego_v(ego.speed * std::cos(ego.yaw), ego.speed * std::sin(ego.yaw));
    const Eigen::Vector2d dp = tr.kf.pos() - ego_p;
    const Eigen::Vector2d dv = tr.kf.vel() - ego_v;
    h.range = dp.norm();
    h.closing_speed = h.range > 1e-3 ? -dp.dot(dv) / h.range : 0.0;

    const Trajectory obj_traj = predict_track(tr, pc_, ego.yaw);
    const Eigen::Vector2d size = tr.footprint();
    h.ttc = first_conflict_time(ego_traj, ego, obj_traj, size, rc_.safety_margin);

    // Classic longitudinal TTC if the object currently overlaps the ego lane laterally.
    const Eigen::Vector2d fwd(std::cos(ego.yaw), std::sin(ego.yaw));
    const Eigen::Vector2d left(-fwd.y(), fwd.x());
    const double lon = dp.dot(fwd), lat = dp.dot(left);
    if (lon > 0.0 && std::abs(lat) < 0.5 * (ego.width + size.y())) {
        const double gap = lon - 0.5 * (ego.length + size.x());
        const double a_rel = tr.kf.acc().dot(fwd) - ego.accel;
        h.ttc_classic = time_to_collision_ca(gap, dv.dot(fwd), a_rel);
    }

    // What-if braking checks (only meaningful if the free-running path conflicts).
    if (std::isfinite(h.ttc)) {
        const double m = rc_.safety_margin;
        h.avoidable_with_margin = !std::isfinite(first_conflict_time(plans.caution, ego, obj_traj, size, m));
        h.driver_can_avoid = !std::isfinite(first_conflict_time(plans.driver, ego, obj_traj, size, m));
        h.aeb_can_avoid = !std::isfinite(first_conflict_time(plans.aeb, ego, obj_traj, size, m));
    }

    // Probability only for candidates whose mean path comes near the ego path.
    bool near = std::isfinite(h.ttc);
    if (!near) {
        const size_t n = std::min(ego_traj.size(), obj_traj.size());
        for (size_t k = 0; k < n && !near; ++k)
            near = (ego_traj[k].pos - obj_traj[k].pos).norm() < rc_.mc_screen_distance + 0.5 * size.x();
    }
    h.collision_probability = near ? collision_probability(tr, ego, ego_traj) : 0.0;

    // An immature track (velocity still uncertain) can only raise CAUTION.
    const double p = h.collision_probability;
    if (std::isfinite(h.ttc)) {
        if (mature && !h.aeb_can_avoid && p >= rc_.p_critical) h.level = WarningLevel::Critical;
        else if (mature && !h.driver_can_avoid && p >= rc_.p_warning) h.level = WarningLevel::Warning;
        else if (!h.avoidable_with_margin && p >= rc_.p_caution) h.level = WarningLevel::Caution;
    }
    return h;
}

std::vector<Hazard> CollisionRiskAssessor::assess(const MultiObjectTracker& tracker,
                                                  const EgoState& ego) const {
    const EgoPlans plans = plan_ego(ego);
    std::vector<Hazard> out;
    for (const Track* tr : tracker.confirmed_tracks())
        out.push_back(assess_track(*tr, ego, plans, tracker.is_mature(*tr)));
    std::sort(out.begin(), out.end(), [](const Hazard& a, const Hazard& b) {
        if (a.level != b.level) return a.level > b.level;
        return a.ttc < b.ttc;
    });
    return out;
}

}  // namespace ipvseas
