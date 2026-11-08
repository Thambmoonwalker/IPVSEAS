// Turns per-frame hazard levels into a stable driver-facing warning with hysteresis:
// a level must persist for a few frames before it is raised, and must be absent for
// longer before it is lowered (prevents flicker and nuisance alerts).
#pragma once

#include <vector>

#include "collision_prediction/collision_risk.hpp"

namespace ipvseas {

struct WarningConfig {
    int raise_frames = 3;       // consecutive frames needed to raise CAUTION / WARNING (0.15 s)
    int raise_frames_critical = 1;
    int lower_frames = 10;      // consecutive frames at a lower level before lowering (0.5 s)
};

struct WarningOutput {
    WarningLevel level = WarningLevel::None;
    int track_id = -1;          // object responsible for the current level
    bool changed = false;
};

class WarningManager {
public:
    explicit WarningManager(WarningConfig cfg = {}) : cfg_(cfg) {}
    WarningOutput update(const std::vector<Hazard>& hazards);
    WarningLevel level() const { return current_; }

private:
    WarningConfig cfg_;
    WarningLevel current_ = WarningLevel::None;
    WarningLevel candidate_ = WarningLevel::None;
    int candidate_count_ = 0;
    int lower_count_ = 0;
    int track_id_ = -1;
};

}  // namespace ipvseas
