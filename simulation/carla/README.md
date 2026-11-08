# Carla

- `record_run.py`: one CARLA 0.9.16 run: scenario (lead_brake, stationary_vehicle, pedestrian_cross,
  free_drive), speed, weather presets + overrides, traffic density, jaywalking, surprise braking
  events, JPG/PNG. Saves `run_config.json` so every run is reproducible.
- `generate_dataset.py`: a whole campaign from a YAML matrix (`experiments/configs/`), with
  resume, dry-run cost estimate, and processing (YOLO → replay → evaluation).
- `make_synthetic_run.py`: the same run-folder format without CARLA, for testing the tool chain.
- `view_run.py`: bird's-eye view of a frame (axis check) and `--radar-check` (radar velocity sign).

Run-folder format and dataset options: `docs/part1_specification.md`, sections 3.4 and 5.6.
