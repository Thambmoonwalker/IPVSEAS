#include "radar/radar_processor.hpp"

#include <algorithm>
#include <numeric>

namespace ipvseas {

namespace {

struct Pt {
    Eigen::Vector3d p;   // vehicle frame
    double vr = 0.0;     // raw range rate
    double vc = 0.0;     // ego-compensated range rate
    double r = 0.0;
};

}  // namespace

Eigen::Vector2d radar_sensor_velocity(const RadarMount& m, double ego_speed, double ego_yaw_rate) {
    // v_sensor = v_ego + w x r_mount  (planar)
    return {ego_speed - ego_yaw_rate * m.y, ego_yaw_rate * m.x};
}

std::vector<RadarObject> process_radar(const std::vector<RadarPoint>& raw, const RadarMount& mount,
                                       const RadarConfig& cfg, double ego_speed, double ego_yaw_rate) {
    const Eigen::Vector2d vs = radar_sensor_velocity(mount, ego_speed, ego_yaw_rate);
    const double cy = std::cos(mount.yaw), sy = std::sin(mount.yaw);

    // 1-3: convert, compensate, filter ghosts.
    std::vector<Pt> pts;
    pts.reserve(raw.size());
    for (const RadarPoint& q : raw) {
        if (!(q.range > 0.5f) || q.range > cfg.max_range) continue;
        const double ce = std::cos(q.elevation);
        const Eigen::Vector3d ps(q.range * ce * std::cos(q.azimuth), q.range * ce * std::sin(q.azimuth),
                                 q.range * std::sin(q.elevation));
        Pt p;
        p.p = {mount.x + cy * ps.x() - sy * ps.y(), mount.y + sy * ps.x() + cy * ps.y(), mount.z + ps.z()};
        if (p.p.z() < cfg.ghost_below_ground) continue;  // reflection via the road surface
        // Unit line of sight in the vehicle frame (horizontal part, as used by the Doppler)
        const Eigen::Vector2d u(cy * ce * std::cos(q.azimuth) - sy * ce * std::sin(q.azimuth),
                                sy * ce * std::cos(q.azimuth) + cy * ce * std::sin(q.azimuth));
        p.vr = q.range_rate;
        p.vc = q.range_rate + vs.dot(u);  // static world point: range_rate = -vs.u  ->  vc = 0
        p.r = q.range;
        pts.push_back(p);
    }

    // 4: DBSCAN on (x, y) with an additional Doppler-agreement condition.
    const int n = static_cast<int>(pts.size());
    std::vector<int> label(n, -2);  // -2 unvisited, -1 noise, >=0 cluster
    // The neighbourhood grows with range: two returns from one object at 190 m are spread by
    // the angular noise (190 m x 0.5 deg = 1.7 m per point), not by the object's size.
    auto neighbours = [&](int i) {
        std::vector<int> nb;
        const double eps = std::max(cfg.cluster_eps, 2.5 * pts[i].r * cfg.sigma_azimuth);
        for (int j = 0; j < n; ++j)
            if ((pts[j].p.head<2>() - pts[i].p.head<2>()).norm() <= eps &&
                std::abs(pts[j].vc - pts[i].vc) <= cfg.cluster_eps_velocity)
                nb.push_back(j);
        return nb;
    };
    int k = 0;
    for (int i = 0; i < n; ++i) {
        if (label[i] != -2) continue;
        std::vector<int> nb = neighbours(i);
        if (static_cast<int>(nb.size()) < cfg.cluster_min_points) { label[i] = -1; continue; }
        label[i] = k;
        std::vector<int> frontier = std::move(nb);
        while (!frontier.empty()) {
            const int j = frontier.back();
            frontier.pop_back();
            if (label[j] == -1) label[j] = k;  // border point
            if (label[j] != -2) continue;
            label[j] = k;
            const std::vector<int> nb2 = neighbours(j);
            if (static_cast<int>(nb2.size()) >= cfg.cluster_min_points)
                for (int x : nb2) frontier.push_back(x);
        }
        ++k;
    }

    // 5-6: cluster statistics and elevation gating.
    std::vector<RadarObject> out;
    for (int c = 0; c < k; ++c) {
        std::vector<int> m;
        for (int i = 0; i < n; ++i)
            if (label[i] == c) m.push_back(i);
        RadarObject o;
        o.num_points = static_cast<int>(m.size());
        Eigen::Vector3d mean = Eigen::Vector3d::Zero();
        double vr = 0.0, vc = 0.0, rr = 0.0;
        for (int i : m) { mean += pts[i].p; vr += pts[i].vr; vc += pts[i].vc; rr += pts[i].r; }
        mean /= o.num_points;
        o.pos = mean.head<2>();
        o.range_rate = vr / o.num_points;
        o.v_comp = vc / o.num_points;
        const double r_mean = rr / o.num_points;
        double var_v = 0.0;
        for (int i : m) var_v += (pts[i].vc - o.v_comp) * (pts[i].vc - o.v_comp);
        o.doppler_spread = std::sqrt(var_v / o.num_points);
        double var_r = 0.0;
        for (int i : m) var_r += (pts[i].vr - o.range_rate) * (pts[i].vr - o.range_rate);
        o.range_rate_spread = std::sqrt(var_r / o.num_points);
        const Eigen::Vector2d rel = o.pos - Eigen::Vector2d(mount.x, mount.y);
        o.range = rel.norm();
        o.azimuth = std::atan2(rel.y(), rel.x());
        o.z_mean = mean.z();
        // Height uncertainty of the mean from elevation noise (dominant at long range).
        o.z_sigma = r_mean * cfg.sigma_elevation / std::sqrt(static_cast<double>(o.num_points));
        o.is_static = std::abs(o.v_comp) < cfg.static_velocity;

        if (o.is_static) {
            // Bottom of a solid object sits near the road; the cluster mean sits between
            // ~0.3 m and the object's mid-height. Compare the 2-sigma interval with the band.
            const double lo = o.z_mean - 2.0 * o.z_sigma, hi = o.z_mean + 2.0 * o.z_sigma;
            const bool inside = lo >= cfg.obstacle_min_height && hi <= cfg.obstacle_max_height;
            const bool outside = hi < cfg.obstacle_min_height || lo > cfg.obstacle_max_height;
            o.height_confirmed = inside;
            o.rejected = outside;
        } else {
            // Moving objects are not road furniture; only reject absurd heights.
            o.height_confirmed = true;
            o.rejected = o.z_mean > cfg.obstacle_max_height + 2.0;
        }
        out.push_back(o);
    }
    std::sort(out.begin(), out.end(), [](const RadarObject& a, const RadarObject& b) { return a.range < b.range; });
    return out;
}

bool radar_object_usable(const RadarObject& o, const RadarMount& mount, const RadarConfig& cfg) {
    return !o.rejected && std::abs(wrap_angle(o.azimuth - mount.yaw)) <= cfg.half_fov - cfg.doppler_fov_margin;
}

Detection radar_object_to_detection(const RadarObject& o, const RadarMount& mount,
                                    const RadarConfig& cfg, double ego_speed, double ego_yaw_rate,
                                    double t) {
    Detection d;
    d.t = t;
    d.pos = o.pos;
    // Polar noise -> Cartesian covariance (range along the line of sight, azimuth across).
    const Eigen::Vector2d u(std::cos(o.azimuth), std::sin(o.azimuth)), l(-u.y(), u.x());
    const double nrm = std::sqrt(static_cast<double>(std::max(1, o.num_points)));
    // Object extent dominates the centroid error, not the per-point noise.
    const double s_r = cfg.sigma_range / nrm + 0.5;
    const double s_l = o.range * cfg.sigma_azimuth / nrm + 0.4;
    d.cov = s_r * s_r * u * u.transpose() + s_l * s_l * l * l.transpose();
    d.cls = ObjectClass::Unknown;
    d.confidence = std::min(1.0, o.num_points / 6.0);
    d.source = Sensor::Radar;
    d.has_range_rate = o.range >= cfg.doppler_min_range;
    d.range_rate = o.range_rate;
    // Measurement noise + geometric / micro-Doppler spread inside the cluster.
    d.range_rate_var = std::pow(cfg.sigma_range_rate, 2) / std::max(1, o.num_points) +
                       std::pow(o.range_rate_spread, 2) + 0.05;
    d.sensor_pos = {mount.x, mount.y};
    d.sensor_vel = radar_sensor_velocity(mount, ego_speed, ego_yaw_rate);
    d.height_confirmed = o.height_confirmed;
    return d;
}

}  // namespace ipvseas
