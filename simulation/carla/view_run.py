"""Bird's-eye view of one frame of a run folder: LiDAR points, ground-truth actors and
(if part1_replay has been run) confirmed tracks, all in the ego VEHICLE frame.

Use it once after your first CARLA recording to check the axis conventions:
a vehicle on your LEFT must appear at +y (up in this plot), and LiDAR points must sit on
the ground-truth boxes.

    python view_run.py ../../datasets/runs/lead_brake_01 --frame 120
    python view_run.py ../../datasets/runs/lead_brake_01 --radar-check

--radar-check: CARLA does not document the sign of its radar velocity. Most radar returns
come from the static world (road, buildings, parked cars); for those the range rate must be
about -v_ego*cos(azimuth)*cos(elevation). The check reports which sign makes that true;
put the result into RADAR_VELOCITY_SIGN in record_run.py (and radar_velocity_sign in calib.txt).
"""

import argparse
import csv
import math
from pathlib import Path

import matplotlib
import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run", type=Path)
    ap.add_argument("--frame", type=int, default=0)
    ap.add_argument("--save", type=Path, help="save to a PNG instead of opening a window")
    ap.add_argument("--radar-check", action="store_true", help="check the radar velocity sign")
    args = ap.parse_args()

    if args.radar_check:
        egos = list(csv.DictReader(open(args.run / "ego.csv")))
        res = {1.0: [], -1.0: []}
        for e in egos:
            v = float(e["speed"])
            path = args.run / "radar" / f"{int(e['frame']):06d}.bin"
            if v < 5.0 or not path.exists():
                continue
            rd = np.fromfile(path, dtype=np.float32).reshape(-1, 5)
            if not len(rd):
                continue
            expected = -v * np.cos(rd[:, 1]) * np.cos(rd[:, 2])
            for s in res:
                res[s].append(np.median(np.abs(s * rd[:, 3] - expected)))
        if not res[1.0]:
            print("no radar frames with ego speed > 5 m/s")
            return
        m_pos, m_neg = float(np.median(res[1.0])), float(np.median(res[-1.0]))
        print(f"median |residual| static-world check: sign +1 -> {m_pos:.2f} m/s, sign -1 -> {m_neg:.2f} m/s")
        print(f"=> use radar_velocity_sign = {'1' if m_pos < m_neg else '-1'}")
        return
    if args.save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    name = f"{args.frame:06d}"
    pts = np.fromfile(args.run / "lidar" / f"{name}.bin", dtype=np.float32).reshape(-1, 4)
    ego = next(r for r in csv.DictReader(open(args.run / "ego.csv")) if int(r["frame"]) == args.frame)
    ex, ey, yaw = float(ego["x"]), float(ego["y"]), float(ego["yaw"])
    c, s = math.cos(yaw), math.sin(yaw)

    def to_vehicle(x, y):
        dx, dy = x - ex, y - ey
        return dx * c + dy * s, -dx * s + dy * c

    fig, ax = plt.subplots(figsize=(10, 6))
    above = pts[:, 2] > 0.3
    ax.scatter(pts[~above, 0], pts[~above, 1], s=0.2, c="0.75", label="ground")
    ax.scatter(pts[above, 0], pts[above, 1], s=0.5, c="k", label="obstacle points")
    ax.add_patch(plt.Rectangle((-2.35, -0.95), 4.7, 1.9, color="tab:blue", alpha=0.6, label="ego"))

    for r in csv.DictReader(open(args.run / "gt_actors.csv")):
        if int(r["frame"]) != args.frame:
            continue
        x, y = to_vehicle(float(r["x"]), float(r["y"]))
        L, W, a = float(r["length"]), float(r["width"]), float(r["yaw"]) - yaw
        corners = np.array([[L / 2, W / 2], [L / 2, -W / 2], [-L / 2, -W / 2], [-L / 2, W / 2], [L / 2, W / 2]])
        Rm = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
        cc = corners @ Rm.T + [x, y]
        ax.plot(cc[:, 0], cc[:, 1], "g-")
        ax.text(x, y + W, f"{r['cls']} {r['actor_id']}", color="g", fontsize=8)

    radar_path = args.run / "radar" / f"{name}.bin"
    if radar_path.exists():
        cal = dict(l.strip().split("=", 1) for l in open(args.run / "calib.txt") if "=" in l and not l.startswith("#"))
        rd = np.fromfile(radar_path, dtype=np.float32).reshape(-1, 5)
        if len(rd):
            rx = float(cal.get("radar_x", 2.4)) + rd[:, 0] * np.cos(rd[:, 2]) * np.cos(rd[:, 1])
            ry = float(cal.get("radar_y", 0.0)) + rd[:, 0] * np.cos(rd[:, 2]) * np.sin(rd[:, 1])
            ax.scatter(rx, ry, s=14, marker="x", c="tab:purple", label="radar")

    tracks = args.run / "tracks.csv"
    if tracks.exists():
        for r in csv.DictReader(open(tracks)):
            if int(r["frame"]) == args.frame:
                x, y = to_vehicle(float(r["x"]), float(r["y"]))
                ax.plot(x, y, "r+", ms=12)
                ax.text(x, y - 1.5, f"T{r['track_id']} {r['cls']}", color="r", fontsize=8)

    ax.set_xlim(-10, 160)
    ax.set_ylim(-25, 25)
    ax.set_aspect("equal")
    ax.set_xlabel("x forward [m]")
    ax.set_ylabel("y left [m]")
    ax.set_title(f"{args.run.name} frame {args.frame} (green = ground truth, red = tracks)")
    ax.legend(loc="lower right", markerscale=2)
    if args.save:
        fig.savefig(args.save, dpi=120)
        print(f"saved {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
