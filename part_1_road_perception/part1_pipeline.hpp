// IPVSEAS Part 1 pipeline, one call per sensor frame:
//   world-frame detections -> tracking -> prediction -> collision risk -> warning
#pragma once

#include <vector>

#include "collision_prediction/collision_risk.hpp"
#include "collision_prediction/warning_manager.hpp"
#include "tracking/tracker.hpp"

namespace ipvseas {

struct Part1Config {
    TrackerConfig tracker;
    PredictionConfig prediction;
    RiskConfig risk;
    WarningConfig warning;
};

struct Part1Output {
    double t = 0.0;
    WarningOutput warning;
    std::vector<Hazard> hazards;  // sorted: most severe first
};

class Part1Pipeline {
public:
    explicit Part1Pipeline(Part1Config cfg = {})
        : cfg_(cfg), tracker_(cfg.tracker), risk_(cfg.risk, cfg.prediction), warner_(cfg.warning) {}

    Part1Output step(double t, const EgoState& ego, const std::vector<Detection>& world_detections) {
        tracker_.update(t, world_detections);
        Part1Output out;
        out.t = t;
        out.hazards = risk_.assess(tracker_, ego);
        out.warning = warner_.update(out.hazards);
        return out;
    }

    const MultiObjectTracker& tracker() const { return tracker_; }
    const Part1Config& config() const { return cfg_; }

private:
    Part1Config cfg_;
    MultiObjectTracker tracker_;
    CollisionRiskAssessor risk_;
    WarningManager warner_;
};

}  // namespace ipvseas
