// Radar fusion (vehicle frame, after camera-LiDAR fusion):
//   * a radar object close to a camera/LiDAR detection adds its Doppler (range rate) to it;
//     the more precise LiDAR position is kept
//   * extra radar clusters of the same object (large trucks split) are absorbed
//   * radar-only objects (beyond LiDAR range, or when LiDAR/camera miss) become
//     "unknown_obstacle" detections with radar position uncertainty and Doppler
#pragma once

#include <vector>

#include "common/types.hpp"

namespace ipvseas {

struct RadarFusionConfig {
    double gate_base = 2.5;     // [m]
    double gate_per_m = 0.02;   // [m per m of range]
};

std::vector<Detection> fuse_radar(const std::vector<Detection>& primary,
                                  const std::vector<Detection>& radar,
                                  const RadarFusionConfig& cfg = {});

}  // namespace ipvseas
