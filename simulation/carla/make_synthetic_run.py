"""Create a run folder in exactly the format record_run.py writes, without CARLA.

Useful to test the whole tool chain (replay -> evaluate -> view) before CARLA is set up,
and as a regression test. The scene: the ego drives along a straight road (at an arbitrary
heading, to exercise the frame transforms) toward a broken-down truck, past a parked car.
LiDAR returns are simulated on the visible faces of each box plus the ground; camera boxes
come from projecting the 3D boxes with the same pinhole model the C++ code uses.

    python make_synthetic_run.py --out ../../datasets/runs/synthetic_truck
"""

import argparse
import csv
import math
from pathlib import Path

import numpy as np

FPS = 20
CAM = dict(width=1280, height=720, fov=90.0, x=1.5, y=0.0, z=1.6)
LIDAR_Z = 2.4
RADAR = dict(x=2.4, y=0.0, z=0.5, range=200.0, half_fov=math.radians(45))


def rot(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


def box_surface_points(cx, cy, length, width, height, rng, spacing=0.12):
    """Points on the faces of an axis-aligned (vehicle-frame) box that face the sensor at the origin."""
    pts = []
    x0, x1, y0, y1 = cx - length / 2, cx + length / 2, cy - width / 2, cy + width / 2
    zs = np.arange(0.3, height, spacing)
    near_x = x0 if cx > 0 else x1                      # face toward the sensor
    for y in np.arange(y0, y1, spacing):
        for z in zs:
            pts.append((near_x, y, z))
    near_y = y0 if cy > 0 else y1                      # side facing the sensor
    if abs(cy) > width / 2:
        for x in np.arange(x0, x1, spacing):
            for z in zs:
                pts.append((x, near_y, z))
    p = np.array(pts, dtype=np.float64)
    if len(p):
        # Sparser with range, like a real spinning LiDAR, plus range noise
        keep = rng.random(len(p)) < np.clip(25.0 / max(1.0, math.hypot(cx, cy)), 0.15, 1.0)
        p = p[keep]
        p[:, :2] += rng.normal(0, 0.02, size=(len(p), 2))
    return p


def project(points_v):
    """Vehicle-frame points (N,3) -> pixel coordinates, same model as CameraModel::from_fov."""
    f = CAM["width"] / (2 * math.tan(math.radians(CAM["fov"]) / 2))
    rel = points_v - np.array([CAM["x"], CAM["y"], CAM["z"]])
    xo, yo, zo = -rel[:, 1], -rel[:, 2], rel[:, 0]
    ok = zo > 0.5
    u = f * xo[ok] / zo[ok] + CAM["width"] / 2
    v = f * yo[ok] / zo[ok] + CAM["height"] / 2
    return u, v


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="../../datasets/runs/synthetic_truck")
    ap.add_argument("--speed", type=float, default=20.0, help="ego speed [m/s]")
    ap.add_argument("--heading-deg", type=float, default=30.0)
    ap.add_argument("--truck-distance", type=float, default=120.0)
    ap.add_argument("--duration", type=float, default=8.0)
    ap.add_argument("--seed", type=int, default=3)
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    out = Path(args.out)
    (out / "lidar").mkdir(parents=True, exist_ok=True)
    (out / "camera").mkdir(parents=True, exist_ok=True)
    (out / "radar").mkdir(parents=True, exist_ok=True)
    yaw = math.radians(args.heading_deg)
    R = rot(yaw)
    start = np.array([500.0, -200.0])
    fwd = R @ np.array([1.0, 0.0])
    # Actors in world frame: (id, cls, along-road distance, lateral offset, length, width, height)
    actors = [(101, "truck", args.truck_distance, 0.0, 9.0, 2.5, 3.2),
              (102, "car", 45.0, -3.2, 4.5, 1.9, 1.5)]
    ego_len, ego_wid, truck_half = 4.7, 1.9, 4.5

    with open(out / "calib.txt", "w") as f:
        f.write("# synthetic run\n")
        f.write(f"fps={FPS}\ncam_width={CAM['width']}\ncam_height={CAM['height']}\ncam_fov={CAM['fov']}\n")
        f.write(f"cam_x={CAM['x']}\ncam_y={CAM['y']}\ncam_z={CAM['z']}\ncam_pitch_deg=0\n")
        f.write(f"lidar_x=0\nlidar_y=0\nlidar_z={LIDAR_Z}\n")
        f.write(f"radar_x={RADAR['x']}\nradar_y={RADAR['y']}\nradar_z={RADAR['z']}\nradar_yaw_deg=0\nradar_velocity_sign=1\n")

    ego_f, gt_f, det_f, col_f = (open(out / n, "w", newline="") for n in
                                 ("ego.csv", "gt_actors.csv", "camera_dets.csv", "collision.csv"))
    ego_w, gt_w, det_w, col_w = csv.writer(ego_f), csv.writer(gt_f), csv.writer(det_f), csv.writer(col_f)
    ego_w.writerow(["frame", "t", "x", "y", "yaw", "speed", "yaw_rate", "accel"])
    gt_w.writerow(["frame", "actor_id", "cls", "x", "y", "yaw", "vx", "vy", "length", "width"])
    det_w.writerow(["frame", "x1", "y1", "x2", "y2", "cls", "conf"])
    col_w.writerow(["frame", "t", "other_actor_id", "other_type"])

    # Ground: rings of points like a 32-beam LiDAR (only the near field returns)
    ground_r = np.concatenate([np.full(720, r) for r in np.geomspace(4.0, 60.0, 24)])
    ground_a = np.tile(np.linspace(-math.pi, math.pi, 720, endpoint=False), 24)
    ground = np.stack([ground_r * np.cos(ground_a), ground_r * np.sin(ground_a), np.zeros_like(ground_r)], 1)

    n = int(args.duration * FPS)
    for i in range(n):
        t = i / FPS
        s_ego = args.speed * t
        ego_xy = start + fwd * s_ego
        ego_w.writerow([i, f"{t:.4f}", f"{ego_xy[0]:.4f}", f"{ego_xy[1]:.4f}", f"{yaw:.6f}",
                        f"{args.speed:.4f}", "0", "0"])
        cloud = [ground + np.c_[rng.normal(0, 0.03, (len(ground), 2)), rng.normal(0, 0.02, len(ground))]]
        radar_targets = []  # vehicle-frame points that reflect radar (all static in this scene)
        collided = False
        for aid, cls, s, lat, L, W, H in actors:
            world = start + fwd * s + R @ np.array([0.0, lat])
            gt_w.writerow([i, aid, cls, f"{world[0]:.3f}", f"{world[1]:.3f}", f"{yaw:.5f}", "0", "0", L, W])
            v = R.T @ (world - ego_xy)  # vehicle frame
            if s - s_ego - (ego_len + L) / 2 <= 0 and abs(lat) < (ego_wid + W) / 2:
                collided = True
            if v[0] > 0:  # radar: reaches beyond the LiDAR (range / FOV checked below)
                near_x = v[0] - L / 2
                for _ in range(max(1, int(round(8 * 50.0 / max(v[0], 5.0))))):
                    radar_targets.append((near_x, v[1] + rng.uniform(-W / 2, W / 2), rng.uniform(0.3, H)))
            if not (0 < v[0] < 100 and abs(v[1]) < 25):
                continue
            pts = box_surface_points(v[0], v[1], L, W, H, rng)
            if len(pts):
                cloud.append(pts)
            corners = np.array([[v[0] + dx, v[1] + dy, z] for dx in (-L / 2, L / 2)
                                for dy in (-W / 2, W / 2) for z in (0.0, H)])
            u, vv = project(corners)
            if len(u) == 8 and v[0] < 80:
                det_w.writerow([i, f"{max(0, u.min()):.1f}", f"{max(0, vv.min()):.1f}",
                                f"{min(CAM['width'] - 1, u.max()):.1f}", f"{min(CAM['height'] - 1, vv.max()):.1f}",
                                cls, "0.85"])
        for _ in range(10):  # static guardrail clutter
            radar_targets.append((rng.uniform(10, 190), rng.choice([-7.0, 7.0]), rng.uniform(0.3, 0.9)))
        rp = []
        for q in radar_targets:
            d = np.array(q) - np.array([RADAR["x"], RADAR["y"], RADAR["z"]])
            r = float(np.linalg.norm(d))
            az = math.atan2(d[1], d[0])
            if r > RADAR["range"] or abs(az) > RADAR["half_fov"] or d[0] <= 0.5:
                continue
            u = d[:2] / r
            rr = float(np.dot(-np.array([args.speed, 0.0]), u))  # static world point
            rp.append((r + rng.normal(0, 0.15), az + rng.normal(0, 0.0087),
                       math.asin(d[2] / r) + rng.normal(0, 0.0087), rr + rng.normal(0, 0.1), 0.0))
        np.array(rp, dtype=np.float32).reshape(-1, 5).tofile(out / "radar" / f"{i:06d}.bin")
        pc = np.concatenate(cloud).astype(np.float32)
        pc = np.c_[pc, np.full(len(pc), 0.5, np.float32)]  # intensity
        pc.tofile(out / "lidar" / f"{i:06d}.bin")
        if collided:
            col_w.writerow([i, f"{t:.4f}", 101, "vehicle.carlamotors.carlacola"])
            print(f"collision at t = {t:.2f} s")
            break
    for f in (ego_f, gt_f, det_f, col_f):
        f.close()
    print(f"wrote {i + 1} frames to {out.resolve()}")


if __name__ == "__main__":
    main()
