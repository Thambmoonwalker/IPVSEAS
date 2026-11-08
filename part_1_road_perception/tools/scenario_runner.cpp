// Runs every synthetic Part 1 scenario and prints the acceptance table.
//   usage: part1_scenarios [--seeds N] [--csv <output_dir>] [--no-radar]
//     --seeds N   Monte Carlo robustness: run each scenario with N sensor-noise seeds
//     --csv dir   write the per-frame log of each scenario (seed 1) to dir/<name>.csv
//     --no-radar  ablation: camera + LiDAR only (shows what the radar contributes)
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include "synthetic/scenarios.hpp"

using namespace ipvseas;

static std::string fmt(double t) {
    if (!std::isfinite(t)) return "   -  ";
    char b[16];
    std::snprintf(b, sizeof(b), "%6.2f", t);
    return b;
}

int main(int argc, char** argv) {
    int seeds = 1;
    std::string csv_dir;
    bool radar = true;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--seeds" && i + 1 < argc) seeds = std::max(1, std::atoi(argv[++i]));
        else if (a == "--csv" && i + 1 < argc) csv_dir = argv[++i];
        else if (a == "--no-radar") radar = false;
    }

    int failed_scenarios = 0;
    std::printf("%-30s %7s %8s %8s %8s %9s %8s\n", "scenario", "passed", "t_coll", "lead_min",
                "lead_med", "FA_frames", "TTC_MAE");
    for (Scenario sc : standard_scenarios()) {
        int passed = 0, fa_frames = 0, mae_n = 0;
        double mae = 0.0, t_coll = kInf;
        std::vector<double> leads;
        for (int s = 1; s <= seeds; ++s) {
            sc.sensor.seed = static_cast<unsigned>(s);
            sc.sensor.radar = radar;
            const ScenarioResult r = run_scenario(sc);
            passed += r.passed ? 1 : 0;
            t_coll = r.t_collision;
            if (r.expect_collision) leads.push_back(r.lead_time);
            else fa_frames += r.frames_at_warning_or_above;
            if (std::isfinite(r.ttc_mae)) { mae += r.ttc_mae; ++mae_n; }
            if (s == 1 && !csv_dir.empty()) std::ofstream(csv_dir + "/" + r.name + ".csv") << r.csv;
        }
        std::sort(leads.begin(), leads.end());
        const std::string lmin = leads.empty() ? "   -  " : fmt(leads.front());
        const std::string lmed = leads.empty() ? "   -  " : fmt(leads[leads.size() / 2]);
        const std::string fa = sc.expect_collision ? "   -" : std::to_string(fa_frames);
        std::printf("%-30s %3d/%-3d %8s %8s %8s %9s %8s\n", sc.name.c_str(), passed, seeds,
                    fmt(t_coll).c_str(), lmin.c_str(), lmed.c_str(), fa.c_str(),
                    mae_n ? fmt(mae / mae_n).c_str() : "   -  ");
        if (passed != seeds) ++failed_scenarios;
    }
    std::printf("\n%d scenario(s) with failures\n", failed_scenarios);
    return failed_scenarios == 0 ? 0 : 1;
}
