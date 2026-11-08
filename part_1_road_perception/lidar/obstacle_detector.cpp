#include "lidar/obstacle_detector.hpp"

#include <algorithm>
#include <cstdint>
#include <queue>
#include <random>
#include <unordered_map>

namespace ipvseas {

namespace {

bool in_roi(const LidarPoint& p, const LidarConfig& c) {
    return p.x >= c.roi_x_min && p.x <= c.roi_x_max && std::abs(p.y) <= c.roi_y_abs;
}

Eigen::Vector3d vec(const LidarPoint& p) { return {p.x, p.y, p.z}; }

std::int64_t cell_key(int ix, int iy) {
    return (static_cast<std::int64_t>(ix) << 32) ^ static_cast<std::uint32_t>(iy);
}

}  // namespace

GroundPlane fit_ground_ransac(const std::vector<LidarPoint>& pts, const LidarConfig& cfg) {
    // Candidate ground points: inside the ROI and in the lower part of the cloud.
    std::vector<int> cand;
    for (int i = 0; i < static_cast<int>(pts.size()); ++i)
        if (in_roi(pts[i], cfg)) cand.push_back(i);
    GroundPlane best;
    if (cand.size() < 3) return best;

    std::mt19937 rng(cfg.seed);
    std::uniform_int_distribution<size_t> pick(0, cand.size() - 1);
    const double cos_tilt = std::cos(cfg.max_ground_tilt_deg * kPi / 180.0);

    for (int it = 0; it < cfg.ransac_iterations; ++it) {
        const Eigen::Vector3d a = vec(pts[cand[pick(rng)]]);
        const Eigen::Vector3d b = vec(pts[cand[pick(rng)]]);
        const Eigen::Vector3d c = vec(pts[cand[pick(rng)]]);
        Eigen::Vector3d n = (b - a).cross(c - a);
        if (n.norm() < 1e-6) continue;
        n.normalize();
        if (n.z() < 0) n = -n;
        if (n.z() < cos_tilt) continue;  // too steep to be the road
        const double d = -n.dot(a);
        int inl = 0;
        for (int idx : cand)
            if (std::abs(n.dot(vec(pts[idx])) + d) < cfg.ground_dist) ++inl;
        // Prefer the LOWEST large plane: a car roof can also be flat.
        if (inl > best.inliers || (inl == best.inliers && d > best.d)) {
            best.normal = n;
            best.d = d;
            best.inliers = inl;
        }
    }
    // Least-squares refinement on the inliers (z = alpha x + beta y + gamma).
    Eigen::MatrixXd A(best.inliers, 3);
    Eigen::VectorXd z(best.inliers);
    int r = 0;
    for (int idx : cand) {
        const Eigen::Vector3d p = vec(pts[idx]);
        if (std::abs(best.height_of(p)) < cfg.ground_dist && r < best.inliers) {
            A.row(r) << p.x(), p.y(), 1.0;
            z(r) = p.z();
            ++r;
        }
    }
    if (r >= 3) {
        const Eigen::Vector3d coef = A.topRows(r).colPivHouseholderQr().solve(z.head(r));
        Eigen::Vector3d n(-coef(0), -coef(1), 1.0);
        const double norm = n.norm();
        best.normal = n / norm;
        best.d = -coef(2) / norm;
    }
    return best;
}

std::vector<LidarObstacle> detect_obstacles(const std::vector<LidarPoint>& pts,
                                            const LidarConfig& cfg, GroundPlane* ground_out) {
    const GroundPlane ground = fit_ground_ransac(pts, cfg);
    if (ground_out) *ground_out = ground;

    // Keep obstacle points: above the ground but below overhead structures.
    std::unordered_map<std::int64_t, std::vector<int>> grid;
    for (int i = 0; i < static_cast<int>(pts.size()); ++i) {
        if (!in_roi(pts[i], cfg)) continue;
        const double h = ground.height_of(vec(pts[i]));
        if (h < cfg.min_height || h > cfg.max_height) continue;
        const int ix = static_cast<int>(std::floor(pts[i].x / cfg.cell_size));
        const int iy = static_cast<int>(std::floor(pts[i].y / cfg.cell_size));
        grid[cell_key(ix, iy)].push_back(i);
    }

    // Connected components over 8-neighbour occupied cells.
    std::unordered_map<std::int64_t, bool> visited;
    std::vector<LidarObstacle> out;
    for (const auto& kv : grid) {
        if (visited[kv.first]) continue;
        std::vector<int> members;
        std::queue<std::int64_t> q;
        q.push(kv.first);
        visited[kv.first] = true;
        while (!q.empty()) {
            const std::int64_t key = q.front();
            q.pop();
            const auto& cell = grid.at(key);
            members.insert(members.end(), cell.begin(), cell.end());
            const int ix = static_cast<int>(key >> 32);
            const int iy = static_cast<int>(static_cast<std::int32_t>(key & 0xffffffff));
            for (int dx = -1; dx <= 1; ++dx)
                for (int dy = -1; dy <= 1; ++dy) {
                    const std::int64_t nk = cell_key(ix + dx, iy + dy);
                    if (grid.count(nk) && !visited[nk]) {
                        visited[nk] = true;
                        q.push(nk);
                    }
                }
        }
        if (static_cast<int>(members.size()) < cfg.min_points) continue;

        // L-shape fit (Zhang et al., 2017, "closeness" criterion): for each orientation
        // (0..90 deg, 1 deg steps) score how close the points lie to the nearest rectangle
        // edges. LiDAR sees an "L" (two faces); min-area and PCA are ambiguous for an L.
        Eigen::Vector2d mean = Eigen::Vector2d::Zero();
        for (int i : members) mean += Eigen::Vector2d(pts[i].x, pts[i].y);
        mean /= static_cast<double>(members.size());
        double best_score = -kInf, yaw = 0.0;
        std::vector<double> p1(members.size()), p2(members.size());
        for (int deg = 0; deg < 90; ++deg) {
            const double th = deg * kPi / 180.0;
            const Eigen::Vector2d f(std::cos(th), std::sin(th)), l(-f.y(), f.x());
            double a0 = kInf, a1 = -kInf, b0 = kInf, b1 = -kInf;
            for (size_t m = 0; m < members.size(); ++m) {
                const Eigen::Vector2d d = Eigen::Vector2d(pts[members[m]].x, pts[members[m]].y) - mean;
                p1[m] = d.dot(f);
                p2[m] = d.dot(l);
                a0 = std::min(a0, p1[m]); a1 = std::max(a1, p1[m]);
                b0 = std::min(b0, p2[m]); b1 = std::max(b1, p2[m]);
            }
            double score = 0.0;
            for (size_t m = 0; m < members.size(); ++m) {
                const double d1 = std::min(a1 - p1[m], p1[m] - a0);
                const double d2 = std::min(b1 - p2[m], p2[m] - b0);
                score += 1.0 / std::max(std::min(d1, d2), 0.05);
            }
            if (score > best_score) {
                best_score = score;
                // Length along the longer side; keep yaw in (-90, 90] deg.
                yaw = (a1 - a0) >= (b1 - b0) ? th : th - kPi / 2;
            }
        }
        const Eigen::Vector2d f(std::cos(yaw), std::sin(yaw)), l(-f.y(), f.x());
        double fmin = kInf, fmax = -kInf, lmin = kInf, lmax = -kInf, hmax = -kInf;
        for (int i : members) {
            const Eigen::Vector2d d = Eigen::Vector2d(pts[i].x, pts[i].y) - mean;
            fmin = std::min(fmin, d.dot(f)); fmax = std::max(fmax, d.dot(f));
            lmin = std::min(lmin, d.dot(l)); lmax = std::max(lmax, d.dot(l));
            hmax = std::max(hmax, ground.height_of(vec(pts[i])));
        }
        LidarObstacle o;
        o.length = std::max(fmax - fmin, 0.2);
        o.width = std::max(lmax - lmin, 0.2);
        if (std::max(o.length, o.width) > cfg.max_cluster_extent) continue;
        const Eigen::Vector2d c2 = mean + f * 0.5 * (fmax + fmin) + l * 0.5 * (lmax + lmin);
        o.height = hmax;
        o.yaw = yaw;
        // Box centre at half height above the local ground.
        const double ground_z = -(ground.normal.x() * c2.x() + ground.normal.y() * c2.y() + ground.d) /
                                ground.normal.z();
        o.center = {c2.x(), c2.y(), ground_z + 0.5 * hmax};
        o.num_points = static_cast<int>(members.size());
        o.point_indices = std::move(members);
        out.push_back(std::move(o));
    }
    std::sort(out.begin(), out.end(), [](const LidarObstacle& a, const LidarObstacle& b) {
        return a.center.head<2>().norm() < b.center.head<2>().norm();
    });
    return out;
}

Detection obstacle_to_detection(const LidarObstacle& o, double t) {
    Detection d;
    d.t = t;
    d.pos = o.center.head<2>();
    // LiDAR sees mostly the near face; centre uncertainty grows with size and range.
    const double range = d.pos.norm();
    const double s = 0.10 + 0.002 * range + 0.05 * std::max(o.length, o.width);
    d.cov = Eigen::Matrix2d::Identity() * s * s;
    d.cls = ObjectClass::Unknown;  // class comes from camera fusion
    d.confidence = std::min(1.0, o.num_points / 30.0);
    d.length = o.length;
    d.width = o.width;
    d.yaw = o.yaw;
    d.source = Sensor::Lidar;
    return d;
}

}  // namespace ipvseas
