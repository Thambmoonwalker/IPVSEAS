# Experiments

- `evaluate_run.py`: metrics for one replayed run (warning lead time, false alarms, tracking
  recall) → `evaluation.json`, optional plot; `--tag no_radar` etc. for sensor ablations.
- `summarize_dataset.py`: one table for a whole campaign (by scenario, speed, weather, traffic
  and sensor configuration).
- `configs/`: campaign definitions for `simulation/carla/generate_dataset.py`
  (`quick_trial`, `v1_campaign`, `weather_and_surprises`, `synthetic_test`).

Experiment plans E-A … E-J: `docs/research_methodology.md`.
