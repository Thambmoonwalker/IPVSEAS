#include "tracking/tracker.hpp"

#include <algorithm>

#include "tracking/hungarian.hpp"

namespace ipvseas {

double Track::vel_sigma() const {
    const Mat6& P = kf.cov();
    return std::sqrt(std::max(P(2, 2), P(3, 3)));
}

Eigen::Vector2d Track::footprint() const {
    Eigen::Vector2d d = default_footprint(cls);
    if (std::isfinite(length) && length > 0.1) d.x() = length;
    if (std::isfinite(width) && width > 0.1) d.y() = width;
    return d;
}

double Track::heading(double fallback) const {
    // Trust the velocity direction only when the speed is well above its uncertainty.
    const Eigen::Vector2d v = kf.vel();
    if (v.norm() > std::max(1.0, 3.0 * vel_sigma())) return std::atan2(v.y(), v.x());
    if (std::isfinite(yaw)) return yaw;
    return fallback;
}

double MultiObjectTracker::sigma_jerk_for(ObjectClass c) {
    switch (group_of(c)) {
        case ClassGroup::Vehicle: return 2.5;  // smooth when steady; IMM in V2 for manoeuvres
        case ClassGroup::VulnerableRoadUser: return 3.0;
        case ClassGroup::Animal: return 2.0;  // animals on roads tend to amble steadily
        default: return 2.5;  // unknown obstacles: mostly LiDAR/radar-only vehicles or debris
    }
}

double MultiObjectTracker::init_vel_sigma_for(ObjectClass c) {
    switch (group_of(c)) {
        case ClassGroup::VulnerableRoadUser: return 2.0;
        case ClassGroup::Animal: return 4.0;
        default: return 15.0;
    }
}

bool MultiObjectTracker::classes_compatible(ObjectClass a, ObjectClass b) {
    const ClassGroup ga = group_of(a), gb = group_of(b);
    return ga == ClassGroup::Unknown || gb == ClassGroup::Unknown || ga == gb;
}

void MultiObjectTracker::start_track(const Detection& d) {
    Track tr;
    tr.id = next_id_++;
    tr.cls = d.cls;
    tr.class_votes[d.cls] += d.confidence;
    tr.kf = KalmanCA(d.pos, d.cov, init_vel_sigma_for(d.cls), 3.0, sigma_jerk_for(d.cls));
    tr.hits = 1;
    tr.first_t = tr.last_t = d.t;
    tr.last_meas = d.pos;
    tr.length = d.length;
    tr.width = d.width;
    tr.yaw = d.yaw;
    tr.radar_height_votes = d.height_confirmed && d.source == Sensor::Radar ? 1 : 0;
    tr.solid_votes = d.height_confirmed && d.source != Sensor::Radar ? 1 : 0;
    tr.height_ok = false;  // needs repeated confirmation (see update)
    tr.confirmed = cfg_.confirm_hits <= 1;
    tracks_.push_back(std::move(tr));
}

void MultiObjectTracker::update(double t, const std::vector<Detection>& detections) {
    const double dt = last_t_ < 0.0 ? 0.0 : t - last_t_;
    last_t_ = t;
    for (Track& tr : tracks_) tr.kf.predict(dt);

    // Cost = Mahalanobis distance^2, forbidden outside the gate or across class groups.
    const int nt = static_cast<int>(tracks_.size());
    const int nd = static_cast<int>(detections.size());
    const double forbidden = cfg_.gate_chi2;
    Eigen::MatrixXd cost = Eigen::MatrixXd::Constant(nt, nd, forbidden * 2.0);
    for (int i = 0; i < nt; ++i)
        for (int j = 0; j < nd; ++j) {
            if (!classes_compatible(tracks_[i].cls, detections[j].cls)) continue;
            const double d2 = tracks_[i].kf.mahalanobis2(detections[j].pos, detections[j].cov);
            if (d2 < cfg_.gate_chi2) cost(i, j) = d2;
        }
    const std::vector<int> assign = hungarian_assign(cost, forbidden);

    std::vector<bool> det_used(nd, false);
    for (int i = 0; i < nt; ++i) {
        Track& tr = tracks_[i];
        const int j = assign[i];
        if (j < 0) { ++tr.misses; continue; }
        const Detection& d = detections[j];
        det_used[j] = true;

        // Two-point velocity initialisation on the first re-observation.
        if (!tr.velocity_initialised && d.t > tr.last_t) {
            const double span = d.t - tr.last_t;
            const Eigen::Vector2d v0 = (d.pos - tr.last_meas) / span;
            const double pos_sigma = std::sqrt(std::max(d.cov(0, 0), d.cov(1, 1)));
            const double vs = std::max(1.0, std::sqrt(2.0) * pos_sigma / span);
            // Only trust it if it is physically plausible for the class.
            const double vmax = group_of(tr.cls) == ClassGroup::VulnerableRoadUser ? 10.0 : 70.0;
            // ...and only if the two-point estimate is reasonably precise.
            if (v0.norm() < vmax && vs < 4.0) tr.kf.set_velocity(v0, vs);
            tr.velocity_initialised = true;
        }
        tr.kf.update(d.pos, d.cov);
        if (d.has_range_rate) {
            tr.last_sensor_pos = d.sensor_pos;
            const Eigen::Vector2d los = d.pos - d.sensor_pos;
            if (los.norm() > 1.0) {
                const double v_world_radial = d.range_rate + d.sensor_vel.dot(los.normalized());
                tr.doppler_static_count = std::abs(v_world_radial) < 0.6 ? tr.doppler_static_count + 1 : 0;
            }
            // Doppler: skip grossly inconsistent values (e.g. a wrongly associated ghost).
            const double pred = tr.kf.predicted_range_rate(d.sensor_pos, d.sensor_vel);
            if (std::abs(d.range_rate - pred) < 3.0 * std::sqrt(d.range_rate_var) + 3.0 * tr.vel_sigma()) {
                tr.kf.update_range_rate(d.range_rate, d.range_rate_var, d.sensor_pos, d.sensor_vel);
                ++tr.radar_updates;
            }
        }
        // A solid road-level object needs 2 LiDAR/camera detections (one random false
        // detection landing on a radar-only track must not confirm it) or 3 radar
        // confirmations (one noisy elevation estimate can make a manhole cover look tall).
        if (d.height_confirmed) {
            if (d.source != Sensor::Radar) ++tr.solid_votes;
            else ++tr.radar_height_votes;
            if (tr.solid_votes >= 2 || tr.radar_height_votes >= 3) tr.height_ok = true;
        }
        tr.last_meas = d.pos;
        tr.last_t = d.t;
        tr.misses = 0;
        ++tr.hits;
        if (tr.hits >= cfg_.confirm_hits) tr.confirmed = true;

        tr.class_votes[d.cls] += d.confidence;
        ObjectClass best = ObjectClass::Unknown;
        double best_w = 0.0;
        for (const auto& [c, w] : tr.class_votes)
            if (c != ObjectClass::Unknown && w > best_w) { best = c; best_w = w; }
        tr.cls = best;  // stays Unknown only if never classified (long-tail obstacle)
        if (std::isfinite(d.length)) tr.length = d.length;
        if (std::isfinite(d.width)) tr.width = d.width;
        if (std::isfinite(d.yaw)) tr.yaw = d.yaw;
    }

    // Merge duplicates (e.g. a long truck split into two radar clusters): if two tracks are
    // statistically at the same place with similar velocity, keep the older one.
    for (size_t i = 0; i < tracks_.size(); ++i)
        for (size_t j = i + 1; j < tracks_.size(); ++j) {
            Track& a = tracks_[i];
            Track& b = tracks_[j];
            if (a.misses > cfg_.max_misses || b.misses > cfg_.max_misses) continue;
            const Eigen::Vector2d dp = a.kf.pos() - b.kf.pos();
            const Eigen::Matrix2d S = a.kf.cov().topLeftCorner<2, 2>() + b.kf.cov().topLeftCorner<2, 2>();
            const double extent = 0.5 * std::max({a.footprint().x(), b.footprint().x(), 1.0});
            const bool close = dp.dot(S.ldlt().solve(dp)) < cfg_.merge_chi2 || dp.norm() < extent;
            if (close && (a.kf.vel() - b.kf.vel()).norm() < 2.0 && classes_compatible(a.cls, b.cls)) {
                Track& loser = a.hits >= b.hits ? b : a;
                Track& keeper = a.hits >= b.hits ? a : b;
                if (keeper.cls == ObjectClass::Unknown) keeper.cls = loser.cls;
                keeper.height_ok = keeper.height_ok || loser.height_ok;
                loser.misses = cfg_.max_misses + 1;  // removed below
            }
        }

    // Delete stale tracks: tentative tracks die after 2 misses, confirmed after max_misses.
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                                 [&](const Track& tr) {
                                     const int limit = tr.confirmed ? cfg_.max_misses : 2;
                                     return tr.misses > limit;
                                 }),
                  tracks_.end());

    for (int j = 0; j < nd; ++j)
        if (!det_used[j]) start_track(detections[j]);
}

std::vector<const Track*> MultiObjectTracker::confirmed_tracks() const {
    std::vector<const Track*> out;
    for (const Track& tr : tracks_)
        if (tr.confirmed) out.push_back(&tr);
    return out;
}

double Track::vel_sigma_radial() const {
    const Eigen::Vector2d d = kf.pos() - last_sensor_pos;
    if (d.norm() < 1.0) return vel_sigma();
    const Eigen::Vector2d u = d.normalized();
    return std::sqrt(std::max(0.0, u.dot(kf.cov().block<2, 2>(2, 2) * u)));
}

double Track::vel_sigma_tangential() const {
    const Eigen::Vector2d d = kf.pos() - last_sensor_pos;
    if (d.norm() < 1.0) return vel_sigma();
    const Eigen::Vector2d l(-d.y() / d.norm(), d.x() / d.norm());
    return std::sqrt(std::max(0.0, l.dot(kf.cov().block<2, 2>(2, 2) * l)));
}

bool Track::doppler_static() const {
    const ClassGroup g = group_of(cls);
    if (g == ClassGroup::VulnerableRoadUser || g == ClassGroup::Animal) return false;
    if (doppler_static_count < 5) return false;
    const Eigen::Vector2d d = kf.pos() - last_sensor_pos;
    if (d.norm() < 1.0) return false;
    const Eigen::Vector2d l(-d.y() / d.norm(), d.x() / d.norm());
    return std::abs(kf.vel().dot(l)) < 3.0 * vel_sigma_tangential();
}

bool MultiObjectTracker::is_mature(const Track& tr) const {
    // Mature = confirmed, velocity well estimated, and currently observed (not coasting
    // on prediction alone, e.g. after leaving the sensor's field of view).
    // A radar-only stationary object of unconfirmed height (manhole cover? bridge?) may
    // raise CAUTION only, until LiDAR / camera / a closer radar look confirms it.
    // With Doppler, closing speed is known precisely even when the sideways speed is not
    // (azimuth noise at long range); the sideways uncertainty then shows up in the Monte
    // Carlo collision probability instead of blocking the warning outright.
    const bool velocity_ok = tr.radar_updates >= 3
                                 ? tr.vel_sigma_radial() < cfg_.mature_vel_sigma &&
                                       tr.vel_sigma_tangential() < cfg_.mature_vel_sigma_tangential
                                 : tr.vel_sigma() < cfg_.mature_vel_sigma;
    return tr.confirmed && velocity_ok && tr.misses <= cfg_.max_coast_for_warning && tr.height_ok;
}

}  // namespace ipvseas
