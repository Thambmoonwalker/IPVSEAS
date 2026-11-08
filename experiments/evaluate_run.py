"""IPVSEAS Part 1: evaluate a replayed run against ground truth.

    python evaluate_run.py ../datasets/runs/lead_brake_01 [--plot]

Results are also written to <run>/evaluation.json (used by summarize_dataset.py).

Reads part1_output.csv and tracks.csv (from part1_replay), collision.csv and gt_actors.csv
(from the recorder) and reports:
  * collision time (the inattentive driver never reacts), first CAUTION / WARNING / CRITICAL
  * warning lead time = t_collision - t_first_warning     (collision runs)
  * frames at WARNING or above                            (no-collision runs = false alarms)
  * tracking: share of nearby ground-truth actors matched by a confirmed track, and
    RMS distance from the track to the actor's footprint
"""

import argparse
import csv
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

LEVELS = {0: "NONE", 1: "CAUTION", 2: "WARNING", 3: "CRITICAL"}


def read(path):
    if not path.exists():
        return []
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def dist_to_box(px, py, a):
    """Distance from a point to a ground-truth actor's footprint (0 if inside). LiDAR tracks
    sit on the visible faces, not the box centre, so centre distance would be misleading."""
    yaw = float(a["yaw"])
    dx, dy = px - float(a["x"]), py - float(a["y"])
    lon = dx * math.cos(yaw) + dy * math.sin(yaw)
    lat = -dx * math.sin(yaw) + dy * math.cos(yaw)
    ox = max(0.0, abs(lon) - float(a["length"]) / 2)
    oy = max(0.0, abs(lat) - float(a["width"]) / 2)
    return math.hypot(ox, oy)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run", type=Path)
    ap.add_argument("--plot", action="store_true", help="save <run>/evaluation.png")
    ap.add_argument("--match-radius", type=float, default=3.0, help="[m] track-to-GT matching")
    ap.add_argument("--tag", default="", help="evaluate part1_output_<tag>.csv (from part1_replay --tag)")
    args = ap.parse_args()
    sfx = f"_{args.tag}" if args.tag else ""

    out = read(args.run / f"part1_output{sfx}.csv")
    if not out:
        sys.exit("no part1_output.csv: run part1_replay first")
    col = read(args.run / "collision.csv")
    ego = {int(r["frame"]): r for r in read(args.run / "ego.csv")}
    t_col = float(col[0]["t"]) if col else math.inf

    first = {}
    warn_frames = 0
    for r in out:
        lvl, t = int(r["level"]), float(r["t"])
        for k in (1, 2, 3):
            if lvl >= k and k not in first:
                first[k] = t
        warn_frames += lvl >= 2

    print(f"run: {args.run}" + (f"  [{args.tag}]" if args.tag else ""))
    print(f"  frames                 : {len(out)}")
    print(f"  collision time         : {t_col:.2f} s" if col else "  collision              : none")
    for k in (1, 2, 3):
        print(f"  first {LEVELS[k]:<16} : " + (f"{first[k]:.2f} s" if k in first else "-"))
    lead = (t_col - first[2]) if (col and 2 in first) else None
    if col:
        print(f"  warning lead time      : " + (f"{lead:.2f} s" if lead is not None else "NO WARNING (miss)"))
    else:
        print(f"  false-alarm frames     : {warn_frames} (WARNING or above, no collision)")
    result = {
        "run": args.run.name, "tag": args.tag or "full", "frames": len(out), "collision": bool(col),
        "t_collision": t_col if col else None, "collided_with": col[0]["other_type"] if col else None,
        "first_caution": first.get(1), "first_warning": first.get(2), "first_critical": first.get(3),
        "lead_time": lead, "missed": bool(col) and lead is None,
        "warning_frames": warn_frames, "false_alarm_frames": 0 if col else warn_frames,
    }

    # Tracking quality: nearby GT actors ahead of the ego vs confirmed tracks
    gt = defaultdict(list)
    for r in read(args.run / "gt_actors.csv"):
        gt[int(r["frame"])].append(r)
    trk = defaultdict(list)
    for r in read(args.run / f"tracks{sfx}.csv"):
        trk[int(r["frame"])].append((float(r["x"]), float(r["y"])))
    n_gt = n_match = 0
    sq = 0.0
    for fr, actors in gt.items():
        if fr not in ego:
            continue
        ex, ey, yaw = float(ego[fr]["x"]), float(ego[fr]["y"]), float(ego[fr]["yaw"])
        for a in actors:
            dx, dy = float(a["x"]) - ex, float(a["y"]) - ey
            lon = dx * math.cos(yaw) + dy * math.sin(yaw)
            lat = -dx * math.sin(yaw) + dy * math.cos(yaw)
            if not (2.0 < lon < 80.0 and abs(lat) < 15.0):  # in the forward sensors' view
                continue
            n_gt += 1
            d = min((dist_to_box(tx, ty, a) for tx, ty in trk.get(fr, [])), default=math.inf)
            if d < args.match_radius:
                n_match += 1
                sq += d * d
    if n_gt:
        print(f"  tracking recall        : {n_match / n_gt:.1%} of {n_gt} GT actor-frames")
        if n_match:
            print(f"  track-to-box distance  : {math.sqrt(sq / n_match):.2f} m RMS (0 = on the object)")
    result["tracking_recall"] = n_match / n_gt if n_gt else None
    result["track_box_rms"] = math.sqrt(sq / n_match) if n_match else None
    (args.run / f"evaluation{sfx}.json").write_text(json.dumps(result, indent=2))

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        t = [float(r["t"]) for r in out]
        fig, ax = plt.subplots(2, 1, figsize=(9, 5), sharex=True)
        ax[0].step(t, [int(r["level"]) for r in out], where="post")
        ax[0].set_yticks(range(4), [LEVELS[k] for k in range(4)])
        ax[0].set_title(f"IPVSEAS Part 1 - {args.run.name}")
        ttc = [float(r["ttc"]) if float(r["ttc"]) >= 0 else float("nan") for r in out]
        ax[1].plot(t, ttc, label="predicted TTC")
        if col:
            ax[1].plot(t, [t_col - x if x <= t_col else float("nan") for x in t], "--", label="true time to impact")
            for a in ax:
                a.axvline(t_col, color="r", lw=1)
        ax[1].set_ylabel("s")
        ax[1].set_xlabel("time [s]")
        ax[1].set_ylim(0, 6)
        ax[1].legend()
        fig.tight_layout()
        fig.savefig(args.run / f"evaluation{sfx}.png", dpi=120)
        print(f"  plot                   : {args.run / f'evaluation{sfx}.png'}")


if __name__ == "__main__":
    main()
