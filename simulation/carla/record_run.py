"""IPVSEAS Part 1: record one CARLA run in the IPVSEAS run-folder format.

The ego vehicle is an "inattentive driver": it follows the lane on autopilot at a fixed
speed but ignores other vehicles and pedestrians, so hazards really lead to collisions.
That lets us measure how early Part 1 warns before the impact the driver would have had.

Everything that defines a run is a command-line option and is saved to run_config.json,
so every run in your dataset is reproducible and its conditions are known:

    python record_run.py --scenario lead_brake --speed 72 --weather rain --traffic medium
    python record_run.py --scenario stationary_vehicle --speed 120 --weather fog --fog-density 70
    python record_run.py --scenario free_drive --duration 60 --traffic heavy --surprise-brakes 4
    python record_run.py --list-weather            # show the weather presets (no CARLA needed)

Scenarios
    lead_brake          lead car 40 m ahead brakes hard at t = 3 s
    stationary_vehicle  broken-down truck 150 m ahead
    pedestrian_cross    pedestrian steps onto the road when the ego is 35 m away
    free_drive          no scripted hazard: background traffic, jaywalkers, surprise events
                        (use it for false-alarm measurements and "unexpected" events)

Unexpected events
    --surprise-brakes N   on average N times per minute, a random NPC vehicle 15-60 m ahead in
                          the ego lane brakes hard for 2-4 s (logged in events.csv)
    --jaywalk P           probability (0-1) that a walking pedestrian crosses the road
                          outside crossings

Output folder (see docs/part1_specification.md, "Run folder format"):
    run_config.json, calib.txt, ego.csv, gt_actors.csv, collision.csv, events.csv,
    camera/<frame>.jpg|png, lidar/<frame>.bin, radar/<frame>.bin

Coordinate convention: CARLA is LEFT-handed (x forward, y right, z up, yaw clockwise).
Everything written here is converted to the right-handed ISO 8855 frame used by the C++
core (x forward, y LEFT, z up, yaw counter-clockwise): y -> -y, yaw -> -yaw.

Written against the CARLA 0.9.16 Python API. Not yet run against a live server: on the
first run, check the LiDAR axis convention with view_run.py (a car on your LEFT must
appear at +y in the bird's-eye view) and the radar velocity sign with
`view_run.py <run> --radar-check` (CARLA does not document the sign convention).
"""

import argparse
import csv
import json
import math
import queue
import random
import sys
import time
from pathlib import Path

import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None

FPS = 20
CAMERA = dict(width=1280, height=720, fov=90.0, x=1.5, y=0.0, z=1.6, pitch_deg=0.0)
LIDAR = dict(channels=32, range=100.0, points_per_second=640000, upper_fov=10.0,
             lower_fov=-30.0, x=0.0, y=0.0, z=2.4)
# Front long-range radar, used as a 4D imaging radar (CARLA reports depth, azimuth, altitude
# and velocity per detection). CARLA's radar is idealised: no noise, ghosts or resolution
# limits; add them in post-processing for weather / robustness experiments (V2).
RADAR = dict(range=200.0, horizontal_fov=90.0, vertical_fov=20.0, points_per_second=12000,
             x=2.4, y=0.0, z=0.5)
RADAR_VELOCITY_SIGN = 1.0  # set by checking with view_run.py --radar-check

# Weather presets: explicit values for carla.WeatherParameters fields (so they mean the same
# thing in every CARLA version). Any field can be overridden on the command line.
# sun_altitude_angle < 0 is night. NOTE: CARLA's fog / rain change the CAMERA image; LiDAR and
# radar are barely affected (see docs) - weather results for those sensors need Part 2 models.
WEATHER_PRESETS = {
    "clear":       dict(cloudiness=5,  precipitation=0,   precipitation_deposits=0,  wind_intensity=10, sun_altitude_angle=70,  fog_density=0,  fog_distance=0,  wetness=0),
    "cloudy":      dict(cloudiness=80, precipitation=0,   precipitation_deposits=0,  wind_intensity=30, sun_altitude_angle=60,  fog_density=0,  fog_distance=0,  wetness=0),
    "rain":        dict(cloudiness=90, precipitation=60,  precipitation_deposits=50, wind_intensity=40, sun_altitude_angle=45,  fog_density=5,  fog_distance=0,  wetness=60),
    "heavy_rain":  dict(cloudiness=100, precipitation=100, precipitation_deposits=90, wind_intensity=80, sun_altitude_angle=30, fog_density=15, fog_distance=0,  wetness=100),
    "fog":         dict(cloudiness=60, precipitation=0,   precipitation_deposits=0,  wind_intensity=5,  sun_altitude_angle=45,  fog_density=60, fog_distance=10, wetness=20),
    "dense_fog":   dict(cloudiness=80, precipitation=0,   precipitation_deposits=0,  wind_intensity=0,  sun_altitude_angle=30,  fog_density=90, fog_distance=2,  wetness=30),
    "sunset_glare": dict(cloudiness=10, precipitation=0,  precipitation_deposits=0,  wind_intensity=10, sun_altitude_angle=5,   fog_density=0,  fog_distance=0,  wetness=0),
    "night":       dict(cloudiness=20, precipitation=0,   precipitation_deposits=0,  wind_intensity=10, sun_altitude_angle=-30, fog_density=0,  fog_distance=0,  wetness=0),
    "night_rain":  dict(cloudiness=90, precipitation=60,  precipitation_deposits=60, wind_intensity=40, sun_altitude_angle=-30, fog_density=10, fog_distance=0,  wetness=80),
    "night_fog":   dict(cloudiness=60, precipitation=0,   precipitation_deposits=0,  wind_intensity=5,  sun_altitude_angle=-30, fog_density=60, fog_distance=10, wetness=30),
}
WEATHER_OVERRIDES = ["cloudiness", "precipitation", "precipitation_deposits", "wind_intensity",
                     "sun_altitude_angle", "sun_azimuth_angle", "fog_density", "fog_distance", "wetness"]

# Background traffic levels: (NPC vehicles, pedestrians). Heavier traffic costs CPU.
TRAFFIC_PRESETS = {"none": (0, 0), "light": (20, 20), "medium": (60, 50), "heavy": (120, 100)}

carla = None  # imported in main() so --help / --list-weather work without CARLA


def iso_xy(loc):
    """CARLA location -> ISO (x, y)."""
    return loc.x, -loc.y


def iso_yaw(rotation):
    return -math.radians(rotation.yaw)


def actor_class(actor):
    tid = actor.type_id
    if tid.startswith("walker."):
        return "pedestrian"
    base = actor.attributes.get("base_type", "").lower()
    if base in ("truck", "bus", "motorcycle", "bicycle", "car"):
        return base
    if any(k in tid for k in ("carlacola", "firetruck", "european_hgv", "ambulance", "sprinter")):
        return "truck"
    if any(k in tid for k in ("harley", "kawasaki", "yamaha", "vespa")):
        return "motorcycle"
    if any(k in tid for k in ("bh.crossbike", "diamondback", "gazelle")):
        return "bicycle"
    return "car"


def centre_world(actor):
    """World location of the bounding-box centre (ISO x, y)."""
    c = actor.get_transform().transform(actor.bounding_box.location)
    return iso_xy(c)


def spawn_ahead(world, bp, ref_wp, distance, lateral=0.0, z=0.5):
    wps = ref_wp.next(distance)
    if not wps:
        return None
    tf = wps[0].transform
    right = tf.get_right_vector()
    tf.location += carla.Location(x=right.x * lateral, y=right.y * lateral, z=z)
    return world.try_spawn_actor(bp, tf)


def resolve_weather(args):
    w = dict(WEATHER_PRESETS[args.weather])
    for k in WEATHER_OVERRIDES:
        v = getattr(args, k)
        if v is not None:
            w[k] = v
    return w


def apply_weather(world, values):
    w = world.get_weather()
    for k, v in values.items():
        if hasattr(w, k):
            setattr(w, k, float(v))
    world.set_weather(w)


def spawn_traffic(client, world, tm, n_vehicles, n_walkers, jaywalk, seed, bp_lib, exclude_tf):
    """Spawn NPC vehicles on autopilot and AI pedestrians. Returns (vehicles, walkers, controllers)."""
    rng = random.Random(seed + 1000)
    vehicles, walkers, controllers = [], [], []
    vbps = [b for b in bp_lib.filter("vehicle.*") if int(b.get_attribute("number_of_wheels")) >= 2]
    spawn_points = world.get_map().get_spawn_points()
    rng.shuffle(spawn_points)
    for sp in spawn_points:
        if len(vehicles) >= n_vehicles:
            break
        if exclude_tf is not None and sp.location.distance(exclude_tf.location) < 15.0:
            continue
        bp = rng.choice(vbps)
        if bp.has_attribute("color"):
            bp.set_attribute("color", rng.choice(bp.get_attribute("color").recommended_values))
        v = world.try_spawn_actor(bp, sp)
        if v:
            v.set_autopilot(True, tm.get_port())
            vehicles.append(v)

    if n_walkers > 0:
        try:
            world.set_pedestrians_seed(seed)
        except AttributeError:
            pass
        world.set_pedestrians_cross_factor(float(jaywalk))
        wbps = bp_lib.filter("walker.pedestrian.*")
        ctrl_bp = bp_lib.find("controller.ai.walker")
        for _ in range(n_walkers * 2):  # some locations fail; try more
            if len(walkers) >= n_walkers:
                break
            loc = world.get_random_location_from_navigation()
            if loc is None:
                continue
            bp = rng.choice(wbps)
            if bp.has_attribute("is_invincible"):
                bp.set_attribute("is_invincible", "false")
            w = world.try_spawn_actor(bp, carla.Transform(loc))
            if w:
                walkers.append(w)
        world.tick()
        for w in walkers:
            c = world.spawn_actor(ctrl_bp, carla.Transform(), attach_to=w)
            controllers.append(c)
        world.tick()
        for c in controllers:
            c.start()
            c.go_to_location(world.get_random_location_from_navigation())
            c.set_max_speed(1.0 + rng.random() * 0.8)
    return vehicles, walkers, controllers


def vehicle_ahead_in_lane(world_map, ego, vehicles, min_d=15.0, max_d=60.0):
    """Nearest NPC vehicle in the ego's lane, between min_d and max_d metres ahead."""
    ego_tf = ego.get_transform()
    ego_wp = world_map.get_waypoint(ego_tf.location)
    fwd = ego_tf.get_forward_vector()
    best, best_d = None, max_d
    for v in vehicles:
        if not v.is_alive:
            continue
        loc = v.get_location()
        d = loc - ego_tf.location
        along = d.x * fwd.x + d.y * fwd.y
        if not (min_d <= along <= best_d):
            continue
        wp = world_map.get_waypoint(loc)
        if wp.road_id == ego_wp.road_id and wp.lane_id == ego_wp.lane_id:
            best, best_d = v, along
    return best


def main():
    global carla
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--tm-port", type=int, default=8000)
    ap.add_argument("--town", default="Town04", help="Town04 has long straight highways")
    ap.add_argument("--scenario", default="lead_brake",
                    choices=["lead_brake", "stationary_vehicle", "pedestrian_cross", "free_drive"])
    ap.add_argument("--speed", type=float, default=72.0, help="ego target speed [km/h]")
    ap.add_argument("--duration", type=float, default=25.0, help="[s]")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--out", default=None)
    # weather
    ap.add_argument("--weather", default="clear", choices=sorted(WEATHER_PRESETS))
    for k in WEATHER_OVERRIDES:
        ap.add_argument("--" + k.replace("_", "-"), dest=k, type=float, default=None,
                        help=f"override weather {k}")
    ap.add_argument("--list-weather", action="store_true", help="print the weather presets and exit")
    # traffic and unexpected events
    ap.add_argument("--traffic", default="none", choices=sorted(TRAFFIC_PRESETS))
    ap.add_argument("--vehicles", type=int, default=None, help="override number of NPC vehicles")
    ap.add_argument("--walkers", type=int, default=None, help="override number of pedestrians")
    ap.add_argument("--jaywalk", type=float, default=0.05, help="pedestrian road-crossing probability 0-1")
    ap.add_argument("--traffic-speed-diff", type=float, default=0.0,
                    help="NPC speed vs limit in %% (positive = slower, negative = faster)")
    ap.add_argument("--surprise-brakes", type=float, default=0.0,
                    help="average sudden-braking events per minute by NPCs ahead in the ego lane")
    # storage
    ap.add_argument("--image-format", default="jpg", choices=["jpg", "png"],
                    help="jpg is ~5-8x smaller; png is lossless")
    ap.add_argument("--jpg-quality", type=int, default=92)
    args = ap.parse_args()

    if args.list_weather:
        for name, vals in WEATHER_PRESETS.items():
            print(f"{name:13s} " + ", ".join(f"{k}={v}" for k, v in vals.items()))
        return

    try:
        import carla as _carla
    except ImportError:
        sys.exit("The 'carla' Python package is not installed. Install the wheel that ships with "
                 "your CARLA build (PythonAPI/carla/dist) or `pip install carla==0.9.16`.")
    carla = _carla

    out = Path(args.out or f"../../datasets/runs/{args.scenario}_{args.seed:02d}").resolve()
    for sub in ("camera", "lidar", "radar"):
        (out / sub).mkdir(parents=True, exist_ok=True)
    random.seed(args.seed)
    rng = random.Random(args.seed)
    weather = resolve_weather(args)
    n_veh, n_walk = TRAFFIC_PRESETS[args.traffic]
    n_veh = args.vehicles if args.vehicles is not None else n_veh
    n_walk = args.walkers if args.walkers is not None else n_walk

    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)
    world = client.get_world()
    if not world.get_map().name.endswith(args.town):
        world = client.load_world(args.town)
    world_map = world.get_map()
    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 1.0 / FPS
    world.apply_settings(settings)
    apply_weather(world, weather)
    night = weather["sun_altitude_angle"] < 0

    tm = client.get_trafficmanager(args.tm_port)
    tm.set_synchronous_mode(True)
    tm.set_random_device_seed(args.seed)
    tm.global_percentage_speed_difference(args.traffic_speed_diff)
    tm.set_global_distance_to_leading_vehicle(2.5)

    bp_lib = world.get_blueprint_library()
    actors, sensors, npc_vehicles, walkers, controllers = [], [], [], [], []
    files = []
    try:
        # ---------------- ego vehicle ----------------
        ego_bp = bp_lib.find("vehicle.lincoln.mkz_2020")
        ego_bp.set_attribute("role_name", "hero")
        spawn_points = world_map.get_spawn_points()
        rng.shuffle(spawn_points)
        ego = None
        for sp in spawn_points:
            ego = world.try_spawn_actor(ego_bp, sp)
            if ego:
                break
        if ego is None:
            sys.exit("could not spawn the ego vehicle")
        actors.append(ego)
        ego.set_autopilot(True, tm.get_port())
        tm.ignore_vehicles_percentage(ego, 100)   # inattentive driver
        tm.ignore_walkers_percentage(ego, 100)
        tm.ignore_lights_percentage(ego, 100)
        tm.auto_lane_change(ego, False)
        tm.set_desired_speed(ego, args.speed)
        if night:
            try:
                ego.set_light_state(carla.VehicleLightState(carla.VehicleLightState.Position |
                                                            carla.VehicleLightState.LowBeam))
            except Exception:
                pass

        # ---------------- background traffic ----------------
        if n_veh or n_walk:
            npc_vehicles, walkers, controllers = spawn_traffic(
                client, world, tm, n_veh, n_walk, args.jaywalk, args.seed, bp_lib, ego.get_transform())
            if night:
                for v in npc_vehicles:
                    try:
                        tm.update_vehicle_lights(v, True)
                    except Exception:
                        pass
            print(f"traffic: {len(npc_vehicles)} vehicles, {len(walkers)} pedestrians")

        # ---------------- sensors ----------------
        cam_bp = bp_lib.find("sensor.camera.rgb")
        cam_bp.set_attribute("image_size_x", str(CAMERA["width"]))
        cam_bp.set_attribute("image_size_y", str(CAMERA["height"]))
        cam_bp.set_attribute("fov", str(CAMERA["fov"]))
        cam_tf = carla.Transform(carla.Location(x=CAMERA["x"], y=-CAMERA["y"], z=CAMERA["z"]),
                                 carla.Rotation(pitch=-CAMERA["pitch_deg"]))
        camera = world.spawn_actor(cam_bp, cam_tf, attach_to=ego)

        lidar_bp = bp_lib.find("sensor.lidar.ray_cast")
        for k in ("channels", "range", "points_per_second", "upper_fov", "lower_fov"):
            lidar_bp.set_attribute(k, str(LIDAR[k]))
        lidar_bp.set_attribute("rotation_frequency", str(FPS))  # one full sweep per tick
        lidar = world.spawn_actor(lidar_bp, carla.Transform(carla.Location(x=LIDAR["x"], z=LIDAR["z"])),
                                  attach_to=ego)

        radar_bp = bp_lib.find("sensor.other.radar")
        for k in ("range", "horizontal_fov", "vertical_fov", "points_per_second"):
            radar_bp.set_attribute(k, str(RADAR[k]))
        radar = world.spawn_actor(radar_bp, carla.Transform(carla.Location(x=RADAR["x"], y=-RADAR["y"], z=RADAR["z"])),
                                  attach_to=ego)

        collision = world.spawn_actor(bp_lib.find("sensor.other.collision"), carla.Transform(), attach_to=ego)
        sensors += [camera, lidar, radar, collision]

        cam_q, lidar_q, radar_q, col_events = queue.Queue(), queue.Queue(), queue.Queue(), []
        camera.listen(cam_q.put)
        lidar.listen(lidar_q.put)
        radar.listen(radar_q.put)
        collision.listen(lambda e: col_events.append(e))

        # Let the ego reach its speed before placing the hazard.
        for _ in range(int(4 * FPS)):
            world.tick()
            for q_ in (cam_q, lidar_q, radar_q):
                while not q_.empty():
                    q_.get()
        col_events.clear()

        # ---------------- scenario actors ----------------
        ego_wp = world_map.get_waypoint(ego.get_location())
        hazard, walker, trigger_done = None, None, False
        if args.scenario == "lead_brake":
            hazard = spawn_ahead(world, bp_lib.find("vehicle.audi.a2"), ego_wp, 40.0)
            if hazard:
                hazard.set_autopilot(True, tm.get_port())
                tm.set_desired_speed(hazard, args.speed)
                tm.auto_lane_change(hazard, False)
        elif args.scenario == "stationary_vehicle":
            hazard = spawn_ahead(world, bp_lib.find("vehicle.carlamotors.carlacola"), ego_wp, 150.0)
            if hazard:
                hazard.apply_control(carla.VehicleControl(hand_brake=True))
        elif args.scenario == "pedestrian_cross":
            lane = ego_wp.lane_width
            walker = spawn_ahead(world, rng.choice(bp_lib.filter("walker.pedestrian.*")), ego_wp,
                                 70.0, lateral=lane / 2 + 2.0, z=1.0)
            hazard = walker
        if hazard:
            actors.append(hazard)
        elif args.scenario != "free_drive":
            print("warning: could not spawn the scenario actor; recording anyway")

        # ---------------- files ----------------
        config = dict(vars(args), out=str(out), weather_values=weather, vehicles_spawned=len(npc_vehicles),
                      walkers_spawned=len(walkers), carla_server=client.get_server_version(),
                      carla_client=client.get_client_version(), town=world_map.name, fps=FPS,
                      camera=CAMERA, lidar=LIDAR, radar=RADAR, recorded_at=time.strftime("%Y-%m-%d %H:%M:%S"),
                      scenario_actor_spawned=hazard is not None)
        (out / "run_config.json").write_text(json.dumps(config, indent=2))
        with open(out / "calib.txt", "w") as f:
            f.write("# IPVSEAS calibration (vehicle frame ISO: x fwd, y left, z up)\n")
            f.write(f"fps={FPS}\n")
            f.write(f"cam_width={CAMERA['width']}\ncam_height={CAMERA['height']}\ncam_fov={CAMERA['fov']}\n")
            f.write(f"cam_x={CAMERA['x']}\ncam_y={CAMERA['y']}\ncam_z={CAMERA['z']}\n")
            f.write(f"cam_pitch_deg={CAMERA['pitch_deg']}\n")
            f.write(f"lidar_x={LIDAR['x']}\nlidar_y={LIDAR['y']}\nlidar_z={LIDAR['z']}\n")
            f.write(f"radar_x={RADAR['x']}\nradar_y={RADAR['y']}\nradar_z={RADAR['z']}\nradar_yaw_deg=0\n")
            f.write(f"radar_velocity_sign={RADAR_VELOCITY_SIGN}\n")

        names = ("ego.csv", "gt_actors.csv", "collision.csv", "events.csv")
        files = [open(out / n, "w", newline="") for n in names]
        ego_w, gt_w, col_w, ev_w = (csv.writer(f) for f in files)
        ego_w.writerow(["frame", "t", "x", "y", "yaw", "speed", "yaw_rate", "accel"])
        gt_w.writerow(["frame", "actor_id", "cls", "x", "y", "yaw", "vx", "vy", "length", "width"])
        col_w.writerow(["frame", "t", "other_actor_id", "other_type"])
        ev_w.writerow(["frame", "t", "event", "actor_id", "detail"])

        n_frames = int(args.duration * FPS)
        t0 = None
        braking = {}  # actor id -> (actor, release time)
        p_brake = args.surprise_brakes / 60.0 / FPS
        last_i = 0
        for i in range(n_frames):
            last_i = i
            frame = world.tick()
            snap = world.get_snapshot()
            t0 = snap.timestamp.elapsed_seconds if t0 is None else t0
            t = snap.timestamp.elapsed_seconds - t0

            # Scripted scenario triggers
            if args.scenario == "lead_brake" and hazard and not trigger_done and t >= 3.0:
                hazard.set_autopilot(False)
                hazard.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
                ev_w.writerow([i, f"{t:.3f}", "lead_brake", hazard.id, "brake=1.0"])
                trigger_done = True
            if args.scenario == "pedestrian_cross" and walker and not trigger_done:
                if walker.get_location().distance(ego.get_location()) < 35.0:
                    left = -walker.get_transform().get_right_vector()  # toward the road centre
                    walker.apply_control(carla.WalkerControl(direction=left, speed=1.4))
                    ev_w.writerow([i, f"{t:.3f}", "pedestrian_start_crossing", walker.id, "speed=1.4"])
                    trigger_done = True

            # Unexpected events: random hard braking by an NPC ahead in the ego lane
            if p_brake > 0 and rng.random() < p_brake:
                v = vehicle_ahead_in_lane(world_map, ego, npc_vehicles)
                if v is not None and v.id not in braking:
                    v.set_autopilot(False)
                    v.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
                    braking[v.id] = (v, t + rng.uniform(2.0, 4.0))
                    ev_w.writerow([i, f"{t:.3f}", "surprise_brake", v.id, f"{v.type_id}"])
            for vid, (v, until) in list(braking.items()):
                if t >= until:
                    if v.is_alive:
                        v.set_autopilot(True, tm.get_port())
                    del braking[vid]

            # Sensors for exactly this frame
            img = cam_q.get(timeout=5.0)
            scan = lidar_q.get(timeout=5.0)
            rad = radar_q.get(timeout=5.0)
            while img.frame < frame:
                img = cam_q.get(timeout=5.0)
            while scan.frame < frame:
                scan = lidar_q.get(timeout=5.0)
            while rad.frame < frame:
                rad = radar_q.get(timeout=5.0)

            name = f"{i:06d}"
            bgra = np.frombuffer(img.raw_data, dtype=np.uint8).reshape(img.height, img.width, 4)
            img_path = out / "camera" / f"{name}.{args.image_format}"
            if cv2 is not None:
                params = [cv2.IMWRITE_JPEG_QUALITY, args.jpg_quality] if args.image_format == "jpg" else []
                cv2.imwrite(str(img_path), bgra[:, :, :3], params)
            else:
                img.save_to_disk(str(img_path))

            pts = np.frombuffer(scan.raw_data, dtype=np.float32).reshape(-1, 4).copy()
            # CARLA sensor frame (left-handed) -> ISO vehicle frame, add the mount offset
            pts[:, 0] += LIDAR["x"]
            pts[:, 1] = -pts[:, 1] + LIDAR["y"]
            pts[:, 2] += LIDAR["z"]
            pts.astype(np.float32).tofile(out / "lidar" / f"{name}.bin")

            # Radar: CARLA azimuth is positive to the RIGHT -> ISO azimuth = -azimuth.
            # The raw velocity is stored unchanged; the replay applies radar_velocity_sign.
            rd = np.frombuffer(rad.raw_data, dtype=np.float32).reshape(-1, 4)  # velocity, azimuth, altitude, depth
            radar_pts = np.stack([rd[:, 3], -rd[:, 1], rd[:, 2], rd[:, 0], np.zeros(len(rd), np.float32)], 1)
            radar_pts.astype(np.float32).tofile(out / "radar" / f"{name}.bin")

            # Ego state (ISO world frame)
            tf, vel = ego.get_transform(), ego.get_velocity()
            ex, ey = centre_world(ego)
            speed = math.hypot(vel.x, vel.y)
            acc = ego.get_acceleration()
            fwd = tf.get_forward_vector()
            ego_w.writerow([i, f"{t:.4f}", f"{ex:.4f}", f"{ey:.4f}", f"{iso_yaw(tf.rotation):.6f}",
                            f"{speed:.4f}", f"{-math.radians(ego.get_angular_velocity().z):.6f}",
                            f"{acc.x * fwd.x + acc.y * fwd.y:.4f}"])

            # Ground truth of nearby actors (for evaluation only)
            ego_loc = ego.get_location()
            for a in list(world.get_actors().filter("vehicle.*")) + list(world.get_actors().filter("walker.pedestrian.*")):
                if a.id == ego.id or a.get_location().distance(ego_loc) > 200.0:
                    continue
                ax, ay = centre_world(a)
                av = a.get_velocity()
                ext = a.bounding_box.extent
                gt_w.writerow([i, a.id, actor_class(a), f"{ax:.3f}", f"{ay:.3f}",
                               f"{iso_yaw(a.get_transform().rotation):.5f}", f"{av.x:.3f}", f"{-av.y:.3f}",
                               f"{2 * ext.x:.2f}", f"{2 * ext.y:.2f}"])

            if col_events:
                for e in col_events:
                    col_w.writerow([i, f"{t:.4f}", e.other_actor.id, e.other_actor.type_id])
                print(f"\ncollision at t = {t:.2f} s with {col_events[0].other_actor.type_id}")
                col_events.clear()
                break  # stop the run at the first impact

            if i % FPS == 0:
                print(f"t = {t:5.1f} s  speed = {speed * 3.6:5.1f} km/h", end="\r")

        print(f"\nrecorded {last_i + 1} frames to {out}")
    finally:
        for f in files:
            f.close()
        for c in controllers:
            try:
                c.stop()
            except Exception:
                pass
        for s in sensors:
            s.stop()
            s.destroy()
        for a in actors + npc_vehicles + walkers + controllers:
            if a.is_alive:
                a.destroy()
        world.apply_settings(original_settings)
        tm.set_synchronous_mode(False)


if __name__ == "__main__":
    main()
