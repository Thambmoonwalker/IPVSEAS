#include "sensor_fusion/radar_fusion.hpp"

#include "tracking/hungarian.hpp"

namespace ipvseas {

std::vector<Detection> fuse_radar(const std::vector<Detection>& primary,
                                  const std::vector<Detection>& radar, const RadarFusionConfig& cfg) {
    auto gate = [&](const Detection& d) { return cfg.gate_base + cfg.gate_per_m * d.pos.norm(); };
    // Radar returns come mostly from the face toward the sensor; a LiDAR box centre of a long
    // vehicle can be metres behind it. Match against the closer of centre and near face.
    auto dist = [](const Detection& r, const Detection& p) {
        double d = (r.pos - p.pos).norm();
        if (std::isfinite(p.length) && p.pos.norm() > 1.0) {
            const Eigen::Vector2d face = p.pos - p.pos.normalized() * 0.5 * p.length;
            d = std::min(d, (r.pos - face).norm());
        }
        return d;
    };

    const double forbidden = 1e6;
    Eigen::MatrixXd cost = Eigen::MatrixXd::Constant(radar.size(), primary.size(), forbidden * 2);
    for (size_t i = 0; i < radar.size(); ++i)
        for (size_t j = 0; j < primary.size(); ++j) {
            const double d = dist(radar[i], primary[j]);
            if (d < gate(primary[j])) cost(i, j) = d;
        }
    const std::vector<int> assign = hungarian_assign(cost, forbidden);

    std::vector<Detection> out = primary;
    for (size_t i = 0; i < radar.size(); ++i) {
        const Detection& r = radar[i];
        if (assign[i] >= 0) {
            Detection& p = out[assign[i]];
            p.has_range_rate = true;
            p.range_rate = r.range_rate;
            p.range_rate_var = r.range_rate_var;
            p.sensor_pos = r.sensor_pos;
            p.sensor_vel = r.sensor_vel;
            p.source = Sensor::Fused;
            continue;
        }
        // Unmatched: absorb if it is just another part of an object already detected.
        bool absorbed = false;
        for (const Detection& p : primary)
        {
            const double half = std::isfinite(p.length) ? 0.5 * p.length : 0.0;
            if (dist(r, p) < gate(p) + half) { absorbed = true; break; }
        }
        if (!absorbed) out.push_back(r);
    }
    return out;
}

}  // namespace ipvseas
