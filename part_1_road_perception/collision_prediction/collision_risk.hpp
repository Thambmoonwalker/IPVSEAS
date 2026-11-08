// Collision-risk assessment: predicted path conflict, time-to-collision, collision
// probability (Monte Carlo over the track's state uncertainty) and warning level.
#pragma once

#include <vector>

#include "behaviour_prediction/trajectory_predictor.hpp"
#include "common/types.hpp"
#include "tracking/tracker.hpp"

namespace ipvseas {

// Warning levels are decided by "what-if" ego braking rollouts against each object's
// predicted trajectory:
//   CAUTION  : a driver reacting after (t_react + caution_margin) at a_driver would collide
//   WARNING  : a driver reacting after t_react at a_driver would collide  -> warn NOW
//   CRITICAL : the system braking after t_system at a_max would collide   -> AEB territory
// For a constant closing speed v this reduces to  TTC <= t_react + v / (2 a_driver).
struct RiskConfig {
    double driver_reaction_time = 1.2;   // [s]
    double driver_decel = 6.0;           // [m/s^2] typical firm braking by a driver
    double system_latency = 0.3;         // [s]
    double max_decel = 9.0;              // [m/s^2]
    double caution_margin = 1.0;         // [s] extra reaction time for the caution level
    double safety_margin = 0.3;          // [m] added around both footprints
    double p_caution = 0.2, p_warning = 0.5, p_critical = 0.7;
    int mc_samples = 200;
    double mc_screen_distance = 6.0;     // [m] skip Monte Carlo if the mean path never gets this close
    unsigned seed = 42;
};

struct Hazard {
    int track_id = -1;
    ObjectClass cls = ObjectClass::Unknown;
    double ttc = kInf;             // first predicted footprint overlap (mean trajectories) [s]
    double ttc_classic = kInf;     // longitudinal TTC for an in-path lead object [s]
    double collision_probability = 0.0;
    double closing_speed = 0.0;    // range rate toward ego [m/s]
    double range = kInf;           // centre distance now [m]
    bool avoidable_with_margin = true;  // driver reacting late (caution check) still avoids it
    bool driver_can_avoid = true;       // normal driver reaction still avoids it
    bool aeb_can_avoid = true;          // automatic max braking still avoids it
    WarningLevel level = WarningLevel::None;
    BehaviourCues cues;
};

// Oriented-rectangle overlap test (separating axis theorem).
bool footprints_overlap(const Eigen::Vector2d& c1, double yaw1, double len1, double wid1,
                        const Eigen::Vector2d& c2, double yaw2, double len2, double wid2);

// First time at which the two trajectories' footprints overlap, or +inf.
double first_conflict_time(const Trajectory& ego, const EgoState& ego_state, const Trajectory& obj,
                           const Eigen::Vector2d& obj_size, double margin);

// Smallest positive root of  gap + v_rel*t + 0.5*a_rel*t^2 = 0  (gap > 0, closing when v_rel < 0).
double time_to_collision_ca(double gap, double v_rel, double a_rel);

class CollisionRiskAssessor {
public:
    CollisionRiskAssessor(RiskConfig rc = {}, PredictionConfig pc = {}) : rc_(rc), pc_(pc) {}

    struct EgoPlans {
        Trajectory free;      // no reaction (constant motion)
        Trajectory caution;   // late driver braking
        Trajectory driver;    // normal driver braking
        Trajectory aeb;       // automatic emergency braking
    };
    EgoPlans plan_ego(const EgoState& ego) const;

    std::vector<Hazard> assess(const MultiObjectTracker& tracker, const EgoState& ego) const;
    Hazard assess_track(const Track& tr, const EgoState& ego, const EgoPlans& plans,
                        bool mature) const;

    const RiskConfig& risk_config() const { return rc_; }
    const PredictionConfig& prediction_config() const { return pc_; }

private:
    double collision_probability(const Track& tr, const EgoState& ego,
                                 const Trajectory& ego_traj) const;
    RiskConfig rc_;
    PredictionConfig pc_;
};

}  // namespace ipvseas
