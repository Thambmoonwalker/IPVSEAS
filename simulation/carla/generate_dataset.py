"""IPVSEAS dataset generator: record a whole matrix of CARLA runs, then (optionally) process
and evaluate every run with several sensor configurations.

    # 1. see what a campaign will cost before running it (no CARLA needed)
    python simulation/carla/generate_dataset.py --config experiments/configs/v1_campaign.yaml --dry-run

    # 2. record it (CARLA server running). Interrupt any time: rerunning resumes.
    python simulation/carla/generate_dataset.py --config experiments/configs/v1_campaign.yaml

    # 3. process: YOLO -> C++ replay (full / no_radar / lidar_only) -> evaluation
    python simulation/carla/generate_dataset.py --config experiments/configs/v1_campaign.yaml --process-only

    # 4. one results table for the whole campaign
    python experiments/summarize_dataset.py datasets/runs/v1_campaign

Config file (YAML): see experiments/configs/v1_campaign.yaml. Each block in `matrix` is expanded
as the cartesian product of its list-valued keys; `seeds: N` means seeds 1..N. Every key is a
record_run.py option (underscores or dashes), e.g. speed, weather, traffic, fog_density,
surprise_brakes, jaywalk, duration.

--backend synthetic replaces CARLA by make_synthetic_run.py (stationary-truck scene only), so
the whole tool chain can be tested without a simulator.
"""

import argparse
import csv
import itertools
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path

try:
    import yaml
except ImportError:
    sys.exit("pip install pyyaml")

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
RECORDER = HERE / "record_run.py"
SYNTHETIC = HERE / "make_synthetic_run.py"
YOLO = REPO / "part_1_road_perception" / "camera" / "detection" / "yolo_detect.py"
EVALUATE = REPO / "experiments" / "evaluate_run.py"

# Sensor configurations evaluated on every recording: tag -> part1_replay flags
CONFIGS = {"full": [], "no_radar": ["--no-radar"], "lidar_only": ["--lidar-only"]}

# Rough storage per frame (JPG camera + 32-channel LiDAR + radar)
MB_PER_FRAME = {"jpg": 0.70, "png": 2.10}


def expand(config):
    """Matrix blocks -> list of (name, options) for every run."""
    defaults = config.get("defaults", {})
    runs, seen = [], set()
    for block in config["matrix"]:
        block = dict(block)
        seeds = block.pop("seeds", None)
        if seeds is not None and "seed" not in block:
            block["seed"] = list(range(1, int(seeds) + 1))
        keys = list(block)
        lists = [v if isinstance(v, list) else [v] for v in block.values()]
        for combo in itertools.product(*lists):
            opts = dict(defaults)
            opts.update(dict(zip(keys, combo)))
            opts.setdefault("seed", 1)
            opts.setdefault("duration", config.get("duration", 25))
            name = run_name(opts)
            if name not in seen:
                seen.add(name)
                runs.append((name, opts))
    return runs


def run_name(o):
    parts = [str(o.get("scenario", "lead_brake")), f"{int(float(o.get('speed', 72)))}kmh",
             str(o.get("weather", "clear")), str(o.get("traffic", "none"))]
    for k in sorted(o):  # any other varied option becomes part of the name
        if k in ("scenario", "speed", "weather", "traffic", "seed", "duration"):
            continue
        parts.append(f"{k.replace('_', '')}{o[k]}")
    parts.append(f"s{int(o['seed']):02d}")
    return "_".join(parts).replace(".", "p").replace("-", "m")


def recorded(run_dir):
    return (run_dir / "ego.csv").exists() and (run_dir / "run_config.json").exists() and \
        sum(1 for _ in open(run_dir / "ego.csv")) > 1


def find_replay(explicit):
    if explicit:
        return Path(explicit)
    exe = "part1_replay.exe" if sys.platform.startswith("win") else "part1_replay"
    candidates = sorted(REPO.glob(f"build/**/part_1_road_perception/**/{exe}"))
    return candidates[0] if candidates else None


def record(name, opts, out_dir, config, backend):
    run_dir = out_dir / name
    if backend == "synthetic":
        cmd = [sys.executable, str(SYNTHETIC), "--out", str(run_dir),
               "--speed", str(float(opts.get("speed", 72)) / 3.6), "--seed", str(opts["seed"]),
               "--truck-distance", str(opts.get("truck_distance", 150)), "--duration", str(opts["duration"])]
    else:
        cmd = [sys.executable, str(RECORDER), "--out", str(run_dir), "--town", config.get("town", "Town04"),
               "--image-format", config.get("image_format", "jpg")]
        for k, v in opts.items():
            cmd += ["--" + k.replace("_", "-"), str(v)]
    print(f"\n=== {name}\n    {' '.join(cmd[1:])}")
    t0 = time.time()
    res = subprocess.run(cmd)
    if res.returncode == 0 and backend == "synthetic":
        cfg = dict(weather="clear", traffic="none")
        cfg.update(opts, backend="synthetic")
        (run_dir / "run_config.json").write_text(json.dumps(cfg, indent=2))
    return res.returncode == 0 and recorded(run_dir), time.time() - t0


def process(run_dir, replay, configs):
    """YOLO (if images and ultralytics available) -> replay per sensor config -> evaluate."""
    images = list((run_dir / "camera").glob("*.jpg")) + list((run_dir / "camera").glob("*.png"))
    if images and not (run_dir / "camera_dets.csv").exists():
        try:
            import ultralytics  # noqa: F401
            subprocess.run([sys.executable, str(YOLO), "--run", str(run_dir)], check=False)
        except ImportError:
            print("    ultralytics not installed: skipping YOLO (camera classes will be missing)")
    results = {}
    for tag in configs:
        cmd = [str(replay), str(run_dir), *CONFIGS[tag]] + ([] if tag == "full" else ["--tag", tag])
        if subprocess.run(cmd, capture_output=True).returncode != 0:
            print(f"    replay failed for {tag}")
            continue
        eval_cmd = [sys.executable, str(EVALUATE), str(run_dir)] + ([] if tag == "full" else ["--tag", tag])
        subprocess.run(eval_cmd, capture_output=True)
        f = run_dir / ("evaluation.json" if tag == "full" else f"evaluation_{tag}.json")
        if f.exists():
            results[tag] = json.loads(f.read_text())
    return results


def write_index(out_dir, rows):
    fields = ["name", "status", "record_seconds", "t_collision", "lead_full", "lead_no_radar",
              "lead_lidar_only", "options"]
    with open(out_dir / "index.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for r in rows.values():
            w.writerow({k: r.get(k, "") for k in fields})


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", type=Path, required=True)
    ap.add_argument("--dry-run", action="store_true", help="list runs, disk and time estimates; record nothing")
    ap.add_argument("--process", action="store_true", help="record, then process + evaluate each run")
    ap.add_argument("--process-only", action="store_true", help="only process already recorded runs")
    ap.add_argument("--configs", default="full,no_radar,lidar_only",
                    help="sensor configurations to evaluate (comma separated)")
    ap.add_argument("--backend", default="carla", choices=["carla", "synthetic"])
    ap.add_argument("--replay-exe", default=None, help="path to part1_replay (auto-detected under build/)")
    ap.add_argument("--limit", type=int, default=None, help="only the first N runs (for a quick trial)")
    ap.add_argument("--sim-fps", type=float, default=10.0,
                    help="assumed recording speed in frames per second, for the time estimate")
    ap.add_argument("--force", action="store_true", help="re-record runs that already exist")
    args = ap.parse_args()

    config = yaml.safe_load(args.config.read_text())
    out_dir = (REPO / config.get("output", f"datasets/runs/{config.get('name', 'campaign')}")).resolve()
    runs = expand(config)[: args.limit]
    fps = 20
    fmt = config.get("image_format", "jpg")
    frames = sum(float(o["duration"]) * fps for _, o in runs)
    print(f"campaign '{config.get('name', args.config.stem)}': {len(runs)} runs -> {out_dir}")
    print(f"  simulated time : {frames / fps / 60:.1f} min ({frames:.0f} frames)")
    print(f"  disk (max)     : {frames * MB_PER_FRAME[fmt] / 1024:.1f} GB with {fmt} images "
          "(runs that end in a collision are shorter)")
    minutes = frames / args.sim_fps / 60
    print(f"  recording time : ~{minutes:.0f} min at {args.sim_fps:.0f} frames/s "
          "(measure your real rate on the first runs and pass --sim-fps)")
    free_gb = shutil.disk_usage(out_dir.parent if out_dir.parent.exists() else REPO).free / 1024 ** 3
    need_gb = frames * MB_PER_FRAME[fmt] / 1024
    print(f"  free disk      : {free_gb:.0f} GB" + ("   <-- NOT ENOUGH: reduce seeds/duration" if need_gb > 0.9 * free_gb else ""))
    if args.dry_run:
        for name, opts in runs:
            print(f"  - {name}")
        return

    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "campaign.yaml").write_text(args.config.read_text())
    configs = [c for c in args.configs.split(",") if c in CONFIGS]
    replay = find_replay(args.replay_exe) if (args.process or args.process_only) else None
    if (args.process or args.process_only) and (replay is None or not replay.exists()):
        sys.exit("part1_replay not found: build the C++ code first or pass --replay-exe")

    rows = {}
    for i, (name, opts) in enumerate(runs, 1):
        run_dir = out_dir / name
        row = dict(name=name, options=json.dumps(opts), status="pending")
        if not args.process_only:
            if recorded(run_dir) and not args.force:
                row["status"] = "recorded (existing)"
            else:
                print(f"[{i}/{len(runs)}]", end="")
                ok, secs = record(name, opts, out_dir, config, args.backend)
                row["status"] = "recorded" if ok else "FAILED"
                row["record_seconds"] = f"{secs:.0f}"
        elif recorded(run_dir):
            row["status"] = "recorded (existing)"
        if (args.process or args.process_only) and recorded(run_dir):
            res = process(run_dir, replay, configs)
            if res:
                row["status"] = "processed"
                full = res.get("full", {})
                row["t_collision"] = full.get("t_collision") or ""
                for tag in CONFIGS:
                    lt = res.get(tag, {}).get("lead_time")
                    row[f"lead_{tag}"] = "" if lt is None else f"{lt:.2f}"
                print(f"    {name}: " + ", ".join(f"{t}={row.get('lead_' + t) or '-'}" for t in configs))
        rows[name] = row
        write_index(out_dir, rows)  # after every run, so progress survives interruptions
    print(f"\nindex: {out_dir / 'index.csv'}")
    print(f"next : python experiments/summarize_dataset.py {out_dir}")


if __name__ == "__main__":
    main()
