#include "collision_prediction/warning_manager.hpp"

namespace ipvseas {

WarningOutput WarningManager::update(const std::vector<Hazard>& hazards) {
    WarningLevel raw = WarningLevel::None;
    int raw_id = -1;
    for (const Hazard& h : hazards)
        if (h.level > raw) { raw = h.level; raw_id = h.track_id; }

    const WarningLevel before = current_;
    if (raw > current_) {
        lower_count_ = 0;
        if (raw == candidate_) ++candidate_count_;
        else { candidate_ = raw; candidate_count_ = 1; }
        const int needed = raw == WarningLevel::Critical ? cfg_.raise_frames_critical
                                                         : cfg_.raise_frames;
        if (candidate_count_ >= needed) {
            current_ = raw;
            track_id_ = raw_id;
            candidate_count_ = 0;
        }
    } else if (raw < current_) {
        candidate_count_ = 0;
        if (++lower_count_ >= cfg_.lower_frames) {
            current_ = raw;
            track_id_ = raw_id;
            lower_count_ = 0;
        }
    } else {
        candidate_count_ = 0;
        lower_count_ = 0;
        if (raw_id >= 0) track_id_ = raw_id;
    }
    return {current_, track_id_, current_ != before};
}

}  // namespace ipvseas
