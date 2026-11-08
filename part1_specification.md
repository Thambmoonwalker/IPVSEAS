# IPVSEAS Part 1: Technical Specification
### Predictive Road-User and Obstacle Collision Prediction (V1)

**Status:** core implemented and tested in synthetic simulation; CARLA recording tools written and ready for the first live run.
**Target:** V1 prototype by the 2nd week of January 2027.

---

## 1. Scope

### 1.1 Problems Part 1 solves

| ID | Problem | V1 |
|---|---|---|
| P1 | Stopped, broken-down or very slow vehicles ahead | ✅ |
| P2 | Sudden braking and cut-ins by other vehicles | ✅ |
| P3 | Pedestrians and animals entering the ego path | ✅ |
| P6 | Long tail: obstacles the camera cannot classify | ✅ (class-agnostic LiDAR obstacles) |

### 1.2 V1 boundaries

- **Sensors:** one front camera + one roof LiDAR. Radar, side/rear sensing and the event camera come in V2/V3.
- **Weather:** clear weather only. Fog, rain and night are Part 2.
- **Output:** driver warnings (NONE / CAUTION / WARNING / CRITICAL). CRITICAL marks "AEB territory"; braking and steering interventions are in the Safety Decision Engine (V2).
- **Simulation only:** no connection to a real vehicle.

---

## 2. Architecture

### 2.1 Processing pipeline (one cycle per sensor frame, 20 Hz)

```text
   CARLA / recorded run (calib, ego pose, camera image, LiDAR sweep)
                 │                                   │
                 ▼                                   ▼
   ┌───────────────────────────┐      ┌──────────────────────────────┐
   │ CAMERA DETECTION (Python) │      │ LIDAR OBSTACLES (C++)        │
   │ YOLO26 → 2D boxes + class │      │ ROI → RANSAC ground → height │
   │ person/vehicle/animal     │      │ band → BEV grid clusters →   │
   └─────────────┬─────────────┘      │ L-shape box (class-agnostic) │
                 │                    └──────────────┬───────────────┘
                 └──────────────┬────────────────────┘
                                ▼
              ┌──────────────────────────────────────┐
              │ CAMERA–LIDAR FUSION (C++)            │
              │ project clusters → IoU → Hungarian   │
              │ matched: LiDAR position + camera cls │
              │ LiDAR only: "unknown_obstacle"       │
              │ camera only: mono ground-plane range │
              └──────────────────┬───────────────────┘
                                 ▼  vehicle → world frame (ego pose)
              ┌──────────────────────────────────────┐
              │ MULTI-OBJECT TRACKING (C++)          │
              │ CA Kalman · Mahalanobis gate ·       │
              │ Hungarian · M-of-N · class voting    │
              └──────────────────┬───────────────────┘
                                 ▼
              ┌──────────────────────────────────────┐
              │ BEHAVIOUR & TRAJECTORY PREDICTION    │
              │ significance-gated motion · stop     │
              │ clamp · covariance propagation ·     │
              │ braking / cut-in / crossing cues     │
              └──────────────────┬───────────────────┘
                                 ▼
              ┌──────────────────────────────────────┐
              │ COLLISION RISK (C++)                 │
              │ footprint conflict (SAT) → TTC ·     │
              │ Monte Carlo collision probability ·  │
              │ what-if braking rollouts → level     │
              └──────────────────┬───────────────────┘
                                 ▼
              ┌──────────────────────────────────────┐
              │ WARNING MANAGER (hysteresis)         │
              └──────────────────┬───────────────────┘
                                 ▼
                 part1_output.csv / live HMI (later)
```

### 2.2 Coordinate frames

| Frame | Definition | Used for |
|---|---|---|
| **Vehicle** (ISO 8855) | origin at the ego centre on the ground; x forward, **y left**, z up | LiDAR points, camera extrinsics, raw detections |
| **World** | fixed; x/y on the ground; yaw counter-clockwise | tracking, prediction, risk |
| **Camera optical** | x right, y down, z forward | projection |
| **CARLA** | **left-handed**: x forward, **y right**, yaw clockwise | converted at recording: `y → −y`, `yaw → −yaw` |

**Why track in the world frame?** A parked truck then has zero velocity no matter how fast the ego moves, so "stationary" is a real, testable property and the motion models stay simple.

### 2.3 Module interfaces

| Type | Fields | Producer → consumer |
|---|---|---|
| `LidarPoint` | x, y, z, intensity (float32, vehicle frame) | recorder → LiDAR detector |
| `CameraDetection` | 2D box, class, confidence | YOLO → fusion |
| `LidarObstacle` | box centre, length, width, height, yaw, point indices | LiDAR → fusion |
| `Detection` | t, world position, 2×2 covariance, class, size, yaw, source | fusion → tracker |
| `Track` | id, CA Kalman state `[x y vx vy ax ay]` + covariance, class votes, hits/misses | tracker → prediction / risk |
| `Hazard` | track id, TTC, classic TTC, collision probability, closing speed, `driver_can_avoid`, `aeb_can_avoid`, level, behaviour cues | risk → warning manager |
| `WarningOutput` | level, responsible track id | warning manager → HMI / log |

### 2.4 Timing budget (20 Hz → 50 ms per cycle)

| Stage | Budget | Measured |
|---|---|---|
| YOLO26-s, 960 px, RTX 5050 laptop | ≤ 20 ms | to measure (`yolo_detect.py` prints ms/frame) |
| LiDAR + fusion + tracking + prediction + risk (C++) | ≤ 10 ms | **2.7 ms/frame**, 17.7 k points, 2-core cloud CPU, file I/O included |
| Margin | ≥ 20 ms | |

---

## 3. Software

### 3.1 Stack

| Layer | Choice | Version / note |
|---|---|---|
| Simulator | CARLA | **0.9.16** (latest UE4 release; supports left-hand traffic maps, relevant for India). 0.10.x (UE5) is heavier for an 8 GB laptop GPU. |
| Detector | Ultralytics YOLO | **YOLO26** (current Ultralytics model), COCO-pretrained `yolo26s.pt` |
| C++ | C++17, Eigen 3.4 | no PCL needed for V1 |
| Python | 3.10–3.12 | `carla`, `ultralytics`, `opencv-python`, `numpy`, `matplotlib` |
| Build | CMake ≥ 3.20 + MSVC (VS 2022) or GCC | `CMakePresets.json`, optional vcpkg |

**CARLA Python client:** it must match the server version (0.9.16). Install the wheel shipped in `PythonAPI/carla/dist/` of your CARLA download, or `pip install carla==0.9.16` if a wheel exists for your Python version. If your Python 3.12 has no matching wheel, create a separate Python 3.10/3.11 virtual environment just for CARLA.

### 3.2 Code map (60 % C++ / 40 % Python)

| File | Lang | Role |
|---|---|---|
| `part_1_road_perception/common/types.hpp`, `geometry.hpp` | C++ | shared types, frame transforms |
| `lidar/obstacle_detector.*` | C++ | RANSAC ground, clustering, L-shape boxes |
| `sensor_fusion/camera_lidar_fusion.*` | C++ | camera model, IoU association, mono range |
| `tracking/kalman_ca.*`, `hungarian.*`, `tracker.*` | C++ | Kalman filter, assignment, track management |
| `behaviour_prediction/trajectory_predictor.*` | C++ | object and ego rollouts, behaviour cues |
| `collision_prediction/collision_risk.*`, `warning_manager.*` | C++ | TTC, probability, levels, hysteresis |
| `part1_pipeline.hpp` | C++ | one-call-per-frame pipeline |
| `synthetic/scenarios.*`, `tools/scenario_runner.cpp` | C++ | synthetic simulator + acceptance runner |
| `tools/replay_run.cpp` | C++ | runs a recorded run folder through everything |
| `tests/test_part1.cpp` | C++ | unit + scenario tests (CTest) |
| `camera/detection/yolo_detect.py` | Py | YOLO on run folders or videos |
| `simulation/carla/record_run.py` | Py | CARLA recorder (inattentive-driver scenarios) |
| `simulation/carla/make_synthetic_run.py` | Py | run folder without CARLA (tool-chain test) |
| `simulation/carla/view_run.py` | Py | bird's-eye-view check of a frame |
| `experiments/evaluate_run.py` | Py | metrics + plot for a replayed run |

### 3.3 Build and run

```bash
# configure + build (choose a preset, see README section 10)
cmake --preset local            # or: cmake --preset vcpkg
cmake --build --preset local

# tests
ctest --test-dir build/local -C Release --output-on-failure

# synthetic acceptance table with 50 noise seeds per scenario
build/local/part_1_road_perception/Release/part1_scenarios --seeds 50
```

On Windows with Visual Studio, executables land in a `Release/` sub-folder as shown; with Ninja or GCC they are directly in `part_1_road_perception/`.

### 3.4 Run-folder format (the contract between Python and C++)

```text
datasets/runs/<run_name>/
├── calib.txt          key=value: fps, cam_width, cam_height, cam_fov, cam_x/y/z, cam_pitch_deg, lidar_x/y/z
├── ego.csv            frame,t,x,y,yaw,speed,yaw_rate,accel              (world, ISO, ego centre)
├── lidar/000123.bin   float32 x,y,z,intensity per point                 (VEHICLE frame, ISO)
├── camera/000123.png  RGB image
├── camera_dets.csv    frame,x1,y1,x2,y2,cls,conf                        (written by yolo_detect.py)
├── gt_actors.csv      frame,actor_id,cls,x,y,yaw,vx,vy,length,width     (evaluation only)
├── collision.csv      frame,t,other_actor_id,other_type                 (empty if no impact)
├── part1_output.csv   frame,t,level,track_id,ttc,p_collision,num_confirmed   (written by part1_replay)
└── tracks.csv         frame,track_id,cls,x,y,vx,vy,mature                    (written by part1_replay)
```

V1 works **offline**: record → detect → replay → evaluate. That keeps experiments reproducible, so algorithm changes can be compared on identical data. A live mode (Python ↔ C++ through pybind11 or a socket) is a V1 stretch goal for the demo video.

---

## 4. Algorithms

### 4.1 Camera detection
- YOLO26-s, COCO-pretrained, input 960 px (larger than the default 640 px to keep distant objects).
- Kept classes and mapping: person → pedestrian; bicycle, car, motorcycle, bus, truck; cat/dog/horse/sheep/cow/elephant/bear/zebra/giraffe → **animal** (COCO's "cow" and "dog" matter on Indian roads).
- Confidence threshold 0.30 at detection, 0.35 at fusion.

### 4.2 LiDAR obstacle detection (class-agnostic)
1. **ROI:** x ∈ [−5, 100] m, |y| ≤ 25 m.
2. **Ground plane by RANSAC** (120 iterations): fit planes through 3 random points; reject planes tilted more than 15°; inliers within 0.20 m; prefer the lowest plane when inlier counts tie (car roofs are flat too); least-squares refinement on the inliers.
3. **Height band:** keep points 0.25–3.5 m above the plane. This removes the road and **overhead structures** (bridges, sign gantries, a classic source of false alarms).
4. **Clustering:** 0.30 m BEV occupancy grid, 8-connected components, at least 8 points, at most 20 m extent (larger clusters are walls).
5. **Box fit:** L-shape fitting with the closeness criterion (Zhang et al., 2017), searching 0–90° in 1° steps. A LiDAR sees an "L" (rear face + near side); minimum-area and PCA fits are ambiguous for an L. In testing, PCA produced a 23° error and a 0.8 m position bias that caused a false alarm; the closeness fit reduced the error to 0.05 m.
6. Every cluster becomes an obstacle **even without a camera class**. This is the V1 answer to the long tail: debris, a fallen load or an unknown vehicle type is still "something solid in my path".

Position uncertainty: σ = 0.10 + 0.002·range + 0.05·size [m].

### 4.3 Camera–LiDAR fusion (late fusion)
- **Camera model:** f = W / (2·tan(FOV/2)), c = (W/2, H/2) (identical to CARLA's camera); vehicle → optical rotation `x_o = −y_v, y_o = −z_v, z_o = x_v`, plus mount pitch.
- Each LiDAR cluster's points are projected and their 2D bounding box is compared with the YOLO boxes. Cost = 1 − IoU, IoU ≥ 0.3 required, optimal assignment by the Hungarian algorithm.
- **Matched:** LiDAR position and size with the camera class (`source = Fused`).
- **LiDAR only:** class `unknown_obstacle`.
- **Camera only:** monocular range from the bottom edge of the box, intersecting the pixel ray with the road plane z = 0. Range uncertainty follows from the pixel error: **σ_Z ≈ Z² / (f·h) · σ_v** (h = camera height, σ_v = 3 px), so camera-only objects at 50 m are uncertain by several metres and the tracker weights them accordingly.

### 4.4 Multi-object tracking
**State** x = [p_x, p_y, v_x, v_y, a_x, a_y]ᵀ in the world frame. **Constant-acceleration model** per axis:

```
F = [1 dt dt²/2; 0 1 dt; 0 0 1]      Q = σ_j² · [dt⁵/20 dt⁴/8 dt³/6; dt⁴/8 dt³/3 dt²/2; dt³/6 dt²/2 dt]
```

σ_j (white jerk) per class: vehicle 2.5, pedestrian/cyclist 3.0, animal 2.0, unknown 6.0 m/s³. The update uses the Joseph form for numerical stability.

- **Gating:** Mahalanobis distance d² = yᵀS⁻¹y < 13.8 (χ², 2 dof, 99.9 %); class groups (vehicle / vulnerable road user / animal) never associate with each other; unknown associates with anything.
- **Assignment:** Hungarian (optimal), cost = d².
- **Initialisation:** position from the first detection; velocity from two consecutive detections when that estimate is precise (σ < 4 m/s), otherwise from the filter.
- **Track management:** confirmed after 3 hits; tentative tracks are deleted after 3 consecutive misses, confirmed tracks after 11 (≈ 0.5 s).
- **Class:** confidence-weighted votes, so a LiDAR-only track becomes "truck" once the camera recognises it. In testing, the truck at 90 m appeared first as `unknown_obstacle`, then became `truck` within the camera's range.
- **Maturity:** a track can raise WARNING or CRITICAL only if it is confirmed, its velocity σ is below 1.5 m/s, and it was measured in the last 2 frames (not coasting on prediction, e.g. after leaving the field of view).

### 4.5 Behaviour and trajectory prediction
Horizon 5 s, step 0.1 s. The key rule is: **only extrapolate motion that is statistically significant.** A filtered acceleration that is pure noise, extrapolated for 3 s, moves an object by metres. In testing, that alone produced most of the false alarms.

- **Stationary test:** vᵀP_v⁻¹v < 9.21 (not distinguishable from 0 at 99 %) or |v| < 0.5 m/s → the object is predicted as stationary.
- **Vehicles:** lateral velocity (relative to the road direction, taken as the ego heading in V1) is kept only if it exceeds 3σ; only the **longitudinal** acceleration is extrapolated, and only if it exceeds 1 m/s².
- **Pedestrians and animals:** constant velocity; sideways motion is kept if it exceeds 2σ (more cautious than for vehicles).
- **Stop clamp:** a braking vehicle stops; it never reverses.
- **Covariance** is propagated with the linear model (P ← FPFᵀ + Q) for display and Monte Carlo.
- **Behaviour cues:** hard braking (longitudinal a < −3 m/s²), cut-in (lateral speed toward the ego > 0.5 m/s), crossing (pedestrian/animal moving toward the ego path > 0.3 m/s).

**Ego prediction:** CTRV (constant turn rate and velocity), plus "what-if" braking rollouts: keep the current motion for a delay, then brake at a given deceleration along the same path.

### 4.6 Collision risk
1. **Footprint conflict:** ego and object are oriented rectangles (plus a 0.3 m margin each); overlap is tested with the separating axis theorem at every prediction step. The first overlap time is the **predicted TTC**.
2. **Classic TTC** for an in-lane lead object: smallest positive root of gap + v_rel·t + ½·a_rel·t² = 0.
3. **Collision probability:** 200 Monte Carlo samples of position and velocity from the track covariance around the significance-gated mean. Each sample is rolled out and checked against the ego path. Acceleration noise is not sampled, otherwise stationary objects "drive away" in many samples; that bug was found on the replay test, where it held the probability for a parked truck at 0.2–0.4.
4. **Warning level from what-if braking rollouts:**

| Level | Condition (with mature track for WARNING/CRITICAL) |
|---|---|
| CRITICAL | system braking after 0.3 s at 9 m/s² still collides, and P ≥ 0.7 |
| WARNING | a driver reacting after 1.2 s and braking at 6 m/s² still collides, and P ≥ 0.5 |
| CAUTION | a driver reacting after 2.2 s still collides, and P ≥ 0.2 |

For a constant closing speed v this reduces to the textbook rule **TTC ≤ t_react + v / (2·a_driver)**. Unlike a fixed TTC threshold, it also handles braking lead vehicles and crossing pedestrians correctly, because it asks the physical question directly: *can a normal driver still avoid this if warned now?*

### 4.7 Warning manager
A level must persist for 3 frames (0.15 s) to be raised (CRITICAL: 1 frame), and must be absent for 10 frames (0.5 s) to be lowered. The 3-frame rule removed rare two-frame false alarms at a cost of 0.05 s of lead time.

### 4.8 Design calculation: how far must Part 1 see?
To warn a normal driver early enough, the obstacle must be confirmed and mature before the gap shrinks to the driver's stopping distance:

d_required ≈ v·t_react + v²/(2·a_driver) + v·t_track + ½(L_ego + L_obj)

with t_react = 1.2 s, a_driver = 6 m/s², t_track ≈ 0.5 s (confirmation + velocity convergence), lengths ≈ 7 m:

| Ego speed | Stopping distance | **Required detection range** |
|---|---|---|
| 40 km/h | 24 m | ~36 m |
| 60 km/h | 43 m | ~59 m |
| 80 km/h | 68 m | ~86 m |
| 100 km/h | 98 m | ~119 m |
| 120 km/h | 133 m | ~156 m |

**Consequence:** a 100 m LiDAR plus a camera with reliable detection to about 80 m covers urban and state-highway speeds. **Expressway speeds need longer range, which is the 4D imaging radar in V2.** This is a quantitative argument for the V2 sensor choice.

---

## 5. Simulation

### 5.1 Two levels of simulation
1. **Synthetic scenario simulator (C++, no CARLA):** exact ground truth, controllable noise, about 10 ms per scenario. Used for algorithm development and as an automated regression test.
2. **CARLA 0.9.16:** realistic rendering, LiDAR physics, traffic and pedestrians. Used for V1 evaluation and the demo video.

### 5.2 The "inattentive driver" concept
In every scenario the ego keeps driving and **does not react** (CARLA Traffic Manager with `ignore_vehicles_percentage = 100` and `ignore_walkers_percentage = 100`). The collision therefore really happens, and its time is the ground truth for **warning lead time = t_collision − t_first_WARNING**. This is the central V1 metric.

### 5.3 CARLA setup (Windows, RTX 5050 laptop)
1. Download the CARLA 0.9.16 Windows package and unzip it (no Unreal build needed).
2. Start the server: `CarlaUE4.exe -quality-level=Low` (Low keeps the frame rate up on an 8 GB GPU; use Epic only for demo footage).
3. Install the matching Python client (section 3.1).
4. `python simulation/carla/record_run.py --scenario lead_brake` (Town04: long highways).
5. **Axis check (once):** `python simulation/carla/view_run.py <run> --frame 100`. Vehicles to the left must appear at +y, and LiDAR points must sit on the green ground-truth boxes. Fix the conversion before anything else if they don't.

### 5.4 Sensor configuration

| Sensor | Setting |
|---|---|
| RGB camera | 1280×720, FOV 90°, at (1.5, 0, 1.6) m |
| LiDAR | 32 channels, 100 m, 640 k points/s, 20 Hz rotation (one sweep per tick), FOV +10°/−30°, at (0, 0, 2.4) m |
| Collision sensor | ends the run and records the impact time |
| World | synchronous mode, fixed Δt = 0.05 s, ClearNoon weather |

### 5.5 Scenario catalogue

| ID | Scenario | Synthetic | CARLA recorder |
|---|---|---|---|
| S1 | Lead vehicle brakes hard (6 m/s²) | ✅ | `lead_brake` |
| S2 | Broken-down truck in lane | ✅ | `stationary_vehicle` |
| S3 | Much slower vehicle ahead | ✅ | (free_drive + traffic) |
| S4 | Cut-in from the adjacent lane | ✅ | V1 stretch |
| S5 | Pedestrian emerging from behind a parked vehicle | ✅ | `pedestrian_cross` |
| S6 | Animal crossing a highway | ✅ | real dashcam video (CARLA has no animals) |
| F1–F5 | Overtaking car, pedestrian on footpath, steady following, parked cars, oncoming traffic | ✅ | `free_drive` |

---

## 6. Testing

### 6.1 Test pyramid

| Level | What | How | Status |
|---|---|---|---|
| Unit | Hungarian, Kalman convergence, ID keeping, clutter rejection, braking stop, TTC formula, SAT overlap, braking-rollout distance (82.08 m at 25 m/s), warning boundary, hysteresis, RANSAC + clustering + gantry rejection, projection, mono range, fusion, frame transforms | `part1_tests` (CTest) | ✅ all pass |
| Scenario acceptance | 11 synthetic scenarios with noise and clutter | `part1_tests`, `part1_scenarios --seeds 50` | ✅ 550/550 |
| Tool chain | synthetic run folder → `part1_replay` → `evaluate_run.py` → `view_run.py` | scripts | ✅ |
| CARLA | recorded scenarios, ≥ 10 runs each | recorder + replay + evaluate | ⏳ next |
| Real video | YOLO on Indian dashcam footage (animals, auto-rickshaws) | `yolo_detect.py --video` | ⏳ |

### 6.2 V1 acceptance criteria

| Metric | Target |
|---|---|
| Warning lead time, S1/S2/S3/S6 | ≥ 1.5 s in every run |
| Warning lead time, S4 (cut-in), S5 (emerging pedestrian) | ≥ 1.0 s in every run |
| False alarms (F1–F5) | 0 frames at WARNING or above |
| TTC error (last 5 s before impact) | ≤ 0.3 s mean absolute |
| Tracking recall (GT actors ahead, ≤ 80 m) | ≥ 90 % |
| C++ processing time | ≤ 10 ms per frame |

### 6.3 Current results: synthetic simulator, 50 noise seeds per scenario

| Scenario | Passed | Impact at | Lead time min / median | TTC error |
|---|---|---|---|---|
| S1 lead vehicle hard braking | 50/50 | 5.40 s | 2.00 / 2.50 s | 0.20 s |
| S2 stationary truck | 50/50 | 5.75 s | 3.15 / 3.20 s | 0.03 s |
| S3 slow vehicle ahead | 50/50 | 3.85 s | 2.10 / 2.70 s | 0.16 s |
| S4 cut-in | 50/50 | 5.10 s | 1.30 / 1.65 s | 0.15 s |
| S5 pedestrian emerging | 50/50 | 3.55 s | 1.00 / 1.50 s | 0.04 s |
| S6 animal crossing | 50/50 | 4.90 s | 2.05 / 2.60 s | 0.04 s |
| F1–F5 false-alarm tests | 50/50 each | – | 0 warning frames | – |

**Replay test** (synthetic run folder, ego at 72 km/h toward a truck 120 m ahead, parked car at the roadside, ego heading 30° to exercise the frame transforms): first WARNING 2.80 s before impact (the physical optimum for a 1.2 s / 6 m/s² driver is about 2.9 s), no false alarm from the parked car, tracking recall 98 %, track on the object (0.00 m RMS), 2.7 ms per frame.

S5 is the hardest case: the pedestrian is hidden until 2.3 s before impact, so the minimum lead time sits exactly at the 1.0 s requirement. This is the motivating case for the event camera (V3) and for occlusion-aware caution (slowing near parked vehicles).

### 6.4 What has not been verified yet
- `record_run.py` has not been run against a live CARLA server. The first run will need the axis check in section 5.3.
- `yolo_detect.py` has not been run here (no GPU); its YOLO26 timing must be measured on the laptop.
- The C++ code has been built and tested with GCC on Linux. The first MSVC build on Windows may raise warnings that need small fixes.
- All results so far are synthetic; CARLA and real-video numbers are the next milestone.

---

## 7. Known limitations and planned improvements

| Limitation | Impact | Improvement |
|---|---|---|
| Single-model CA filter | trades smoothness against manoeuvre response | **IMM** (stationary / CV / CA) in V2 |
| Road direction = ego heading | lateral gating is approximate on curves and at junctions | lane geometry from the CARLA map / lane detection |
| LiDAR box only covers visible faces | object length underestimated head-on | class-based size prior from fusion |
| Ego pose from simulator ground truth | real cars need odometry | IMU + wheel-speed EKF (V2) |
| Front-only sensing | coasting tracks beside the car can only raise CAUTION | Part 2: 360° radar and blind-spot coverage |
| Late, box-level fusion | loses information in heavy occlusion | BEV / mid-level fusion research (V3) |
| Occluded pedestrians | S5 lead time is the minimum (1.0 s) | occlusion-aware caution, event camera (V3) |

---

## 8. Milestones to V1 (target: 2nd week of January 2027)

| Week | Dates | Deliverable |
|---|---|---|
| 1 | 9–15 Nov | Build on Windows (MSVC), all tests green; CARLA 0.9.16 running; synthetic run → replay → evaluate on the laptop |
| 2 | 16–22 Nov | First CARLA recordings (`lead_brake`, `stationary_vehicle`); axis check; YOLO on recorded frames; first replays |
| 3 | 23–29 Nov | Tune LiDAR detection on CARLA data (ground, height band, cluster limits); tracking recall ≥ 90 % |
| 4 | 30 Nov–6 Dec | `pedestrian_cross` + free-drive false-alarm runs; 10 runs per scenario; first results table |
| 5 | 7–13 Dec | Cut-in scenario in CARLA; YOLO on Indian dashcam video (animals, auto-rickshaws) |
| 6 | 14–20 Dec | Live mode for the demo (pybind11 or socket), on-screen warning overlay |
| 7 | 21–27 Dec | Evaluation campaign; plots; compare against a detection-only baseline (distance threshold) |
| 8 | 28 Dec–3 Jan | Demo video, README results, documentation |
| 9 | 4–10 Jan | Buffer, fixes, **V1 release tag** |

---

## 9. References
- Zhang, X. et al. (2017). *Efficient L-Shape Fitting for Vehicle Detection Using Laser Scanners.* IEEE Intelligent Vehicles Symposium.
- Bar-Shalom, Y., Li, X. R., Kirubarajan, T. (2001). *Estimation with Applications to Tracking and Navigation.* Wiley. (CA model, gating, IMM)
- Kuhn, H. W. (1955). *The Hungarian Method for the Assignment Problem.*
- CARLA Simulator documentation, release 0.9.16.
- Ultralytics YOLO26 documentation.
