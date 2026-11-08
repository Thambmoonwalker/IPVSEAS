# IPVSEAS Part 1: Technical Specification
### Predictive Road-User and Obstacle Collision Prediction (V1)

**Status:** camera + LiDAR + 4D imaging radar core implemented and tested in synthetic simulation (700/700 scenario runs pass); CARLA recording tools written and ready for the first live run.
**Target:** V1 prototype by the 2nd week of January 2027.

---

## 1. Scope

### 1.1 Problems Part 1 solves

| ID | Problem | V1 |
|---|---|---|
| P1 | Stopped, broken-down or very slow vehicles ahead, including at expressway speed | ✅ (radar extends the range to 200 m) |
| P2 | Sudden braking and cut-ins by other vehicles | ✅ |
| P3 | Pedestrians and animals entering the ego path | ✅ |
| P6 | Long tail: obstacles the camera cannot classify | ✅ (class-agnostic LiDAR and radar obstacles) |

### 1.2 V1 boundaries

- **Sensors:** one front camera, one roof LiDAR (100 m) and one front 4D imaging radar (long-range mode, 200 m, ±45°). Side/rear sensing and the event camera come in V2/V3.
- **Weather:** clear weather only. Fog, rain and night are Part 2 (where radar becomes the backbone sensor).
- **Output:** driver warnings (NONE / CAUTION / WARNING / CRITICAL). CRITICAL marks "AEB territory"; braking and steering interventions are in the Safety Decision Engine (V2).
- **Simulation only:** no connection to a real vehicle.

---

## 2. Architecture

### 2.1 Processing pipeline (one cycle per sensor frame, 20 Hz)

```text
      CARLA / recorded run (calib, ego pose, camera image, LiDAR sweep, radar detections)
           │                          │                               │
           ▼                          ▼                               ▼
 ┌────────────────────┐   ┌──────────────────────────┐   ┌──────────────────────────────┐
 │ CAMERA (Python)    │   │ LIDAR OBSTACLES (C++)    │   │ 4D RADAR (C++)               │
 │ YOLO26 → 2D boxes  │   │ ROI → RANSAC ground →    │   │ polar → Cartesian → ego      │
 │ + class            │   │ height band → BEV grid → │   │ Doppler compensation → ghost │
 │                    │   │ L-shape boxes            │   │ filter → range-scaled DBSCAN │
 └─────────┬──────────┘   └────────────┬─────────────┘   │ → elevation gating           │
           └──────────────┬────────────┘                 └──────────────┬───────────────┘
                          ▼                                             │
            ┌──────────────────────────────┐                            │
            │ CAMERA–LIDAR FUSION          │                            │
            │ IoU + Hungarian; LiDAR-only →│                            │
            │ unknown; camera-only → mono  │                            │
            └──────────────┬───────────────┘                            │
                           └───────────────┬────────────────────────────┘
                                           ▼
                         ┌──────────────────────────────────────┐
                         │ RADAR FUSION                         │
                         │ match near face → add Doppler;       │
                         │ absorb split clusters; radar-only →  │
                         │ long-range unknown obstacle          │
                         └──────────────────┬───────────────────┘
                                            ▼  vehicle → world frame (ego pose)
                         ┌──────────────────────────────────────┐
                         │ MULTI-OBJECT TRACKING                │
                         │ CA Kalman + EKF Doppler update ·     │
                         │ gating · Hungarian · M-of-N · class  │
                         │ votes · height confirmation · merge  │
                         └──────────────────┬───────────────────┘
                                            ▼
                         ┌──────────────────────────────────────┐
                         │ BEHAVIOUR & TRAJECTORY PREDICTION    │
                         │ significance-gated motion · stop     │
                         │ clamp · Doppler-stationary model     │
                         └──────────────────┬───────────────────┘
                                            ▼
                         ┌──────────────────────────────────────┐
                         │ COLLISION RISK                       │
                         │ footprint conflict → TTC · Monte     │
                         │ Carlo probability · what-if braking  │
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
| **Vehicle** (ISO 8855) | origin at the ego centre on the ground; x forward, **y left**, z up | LiDAR points, radar detections, camera extrinsics, raw detections |
| **World** | fixed; x/y on the ground; yaw counter-clockwise | tracking, prediction, risk |
| **Camera optical** | x right, y down, z forward | projection |
| **Radar sensor** | polar: range, azimuth (**positive left**), elevation (positive up), range rate (**positive = moving away**) | raw radar detections |
| **CARLA** | **left-handed**: x forward, **y right**, yaw clockwise, radar azimuth positive right | converted at recording: `y → −y`, `yaw → −yaw`, `azimuth → −azimuth` |

**Why track in the world frame?** A parked truck then has zero velocity no matter how fast the ego moves, so "stationary" is a real, testable property. This is also what makes radar Doppler useful: after compensating for the ego's own motion, a stationary object has zero Doppler.

### 2.3 Module interfaces

| Type | Fields | Producer → consumer |
|---|---|---|
| `LidarPoint` | x, y, z, intensity (float32, vehicle frame) | recorder → LiDAR detector |
| `RadarPoint` | range, azimuth, elevation, range rate, RCS (float32, sensor frame) | recorder → radar processor |
| `CameraDetection` | 2D box, class, confidence | YOLO → fusion |
| `LidarObstacle` | box centre, length, width, height, yaw, point indices | LiDAR → fusion |
| `RadarObject` | centroid, mean Doppler (raw + ego-compensated), height ± σ, micro-Doppler spread, static / height-confirmed / rejected flags | radar → radar fusion |
| `Detection` | t, world position + 2×2 covariance, class, size, yaw, source, **optional range rate + radar position/velocity**, height-confirmed flag | fusion → tracker |
| `Track` | id, CA Kalman state `[x y vx vy ax ay]` + covariance, class votes, hits/misses, height confirmation, Doppler-stationary count | tracker → prediction / risk |
| `Hazard` | track id, TTC, classic TTC, collision probability, closing speed, `driver_can_avoid`, `aeb_can_avoid`, level, behaviour cues | risk → warning manager |
| `WarningOutput` | level, responsible track id | warning manager → HMI / log |

### 2.4 Timing budget (20 Hz → 50 ms per cycle)

| Stage | Budget | Measured |
|---|---|---|
| YOLO26-s, 960 px, RTX 5050 laptop | ≤ 20 ms | to measure (`yolo_detect.py` prints ms/frame) |
| LiDAR + radar + fusion + tracking + prediction + risk (C++) | ≤ 10 ms | **2.0–2.3 ms/frame** (≈18 k LiDAR points + radar, 2-core cloud CPU, file I/O included) |
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

### 3.2 Code map

| File | Lang | Role |
|---|---|---|
| `part_1_road_perception/common/types.hpp`, `geometry.hpp` | C++ | shared types, frame transforms |
| `lidar/obstacle_detector.*` | C++ | RANSAC ground, clustering, L-shape boxes |
| `radar/radar_processor.*` | C++ | 4D radar: Doppler compensation, ghosts, DBSCAN, elevation gating |
| `sensor_fusion/camera_lidar_fusion.*` | C++ | camera model, IoU association, mono range |
| `sensor_fusion/radar_fusion.*` | C++ | radar ↔ camera/LiDAR association, Doppler hand-over |
| `tracking/kalman_ca.*`, `hungarian.*`, `tracker.*` | C++ | Kalman filter + EKF Doppler update, assignment, track management |
| `behaviour_prediction/trajectory_predictor.*` | C++ | object and ego rollouts, behaviour cues |
| `collision_prediction/collision_risk.*`, `warning_manager.*` | C++ | TTC, probability, levels, hysteresis |
| `part1_pipeline.hpp` | C++ | one-call-per-frame pipeline |
| `synthetic/scenarios.*`, `tools/scenario_runner.cpp` | C++ | synthetic simulator (incl. radar model) + acceptance runner |
| `tools/replay_run.cpp` | C++ | runs a recorded run folder through everything |
| `tests/test_part1.cpp` | C++ | unit + scenario tests (CTest) |
| `camera/detection/yolo_detect.py` | Py | YOLO on run folders or videos |
| `simulation/carla/record_run.py` | Py | CARLA recorder (camera, LiDAR, radar; scenarios, weather, traffic, surprise events) |
| `simulation/carla/generate_dataset.py` | Py | campaign generator: parameter matrix → record → process → evaluate |
| `experiments/summarize_dataset.py`, `experiments/configs/*.yaml` | Py | campaign summary table; campaign definitions |
| `simulation/carla/make_synthetic_run.py` | Py | run folder without CARLA (tool-chain test) |
| `simulation/carla/view_run.py` | Py | bird's-eye-view check of a frame; radar velocity-sign check |
| `experiments/evaluate_run.py` | Py | metrics + plot for a replayed run |

### 3.3 Build and run

```bash
# configure + build (choose a preset, see README section 10)
cmake --preset local            # or: cmake --preset vcpkg
cmake --build --preset local

# tests (unit tests + 20-seed scenario robustness)
ctest --test-dir build/local -C Release --output-on-failure

# synthetic acceptance table, 50 noise seeds per scenario; and the radar ablation
build/local/part_1_road_perception/Release/part1_scenarios --seeds 50
build/local/part_1_road_perception/Release/part1_scenarios --seeds 50 --no-radar
```

On Windows with Visual Studio, executables land in a `Release/` sub-folder as shown; with Ninja or GCC they are directly in `part_1_road_perception/`.

### 3.4 Run-folder format (the contract between Python and C++)

```text
datasets/runs/<run_name>/
├── calib.txt          key=value: fps, cam_width, cam_height, cam_fov, cam_x/y/z, cam_pitch_deg,
│                      lidar_x/y/z, radar_x/y/z, radar_yaw_deg, radar_velocity_sign
├── ego.csv            frame,t,x,y,yaw,speed,yaw_rate,accel              (world, ISO, ego centre)
├── lidar/000123.bin   float32 x,y,z,intensity per point                 (VEHICLE frame, ISO)
├── radar/000123.bin   float32 range,azimuth,elevation,range_rate,rcs    (SENSOR frame, ISO azimuth)
├── camera/000123.jpg  RGB image (or .png)
├── camera_dets.csv    frame,x1,y1,x2,y2,cls,conf                        (written by yolo_detect.py)
├── gt_actors.csv      frame,actor_id,cls,x,y,yaw,vx,vy,length,width     (evaluation only)
├── collision.csv      frame,t,other_actor_id,other_type                 (empty if no impact)
├── events.csv         frame,t,event,actor_id,detail                     (scripted + surprise events)
├── run_config.json    every option of the run (scenario, weather values, traffic, seed, versions)
├── part1_output.csv   frame,t,level,track_id,ttc,p_collision,num_confirmed   (written by part1_replay)
├── tracks.csv         frame,track_id,cls,x,y,vx,vy,mature                    (written by part1_replay)
└── evaluation.json    metrics (evaluate_run.py); *_no_radar / *_lidar_only variants via --tag
```

V1 works **offline**: record → detect → replay → evaluate. That keeps experiments reproducible, so algorithm changes (or sensor ablations: `--no-radar`, `--lidar-only`) can be compared on identical data. A live mode (Python ↔ C++ through pybind11 or a socket) is a V1 stretch goal for the demo video.

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

### 4.4 4D imaging radar
Each radar detection gives **range r, azimuth θ, elevation φ and radial velocity (range rate) ṙ**. Sensor model: front bumper (x = 2.3–2.4 m, z = 0.5 m), 200 m, ±45°, σ_r = 0.15 m, σ_θ = σ_φ = 0.5°, σ_ṙ = 0.1 m/s.

1. **Polar → Cartesian** in the vehicle frame: x = r·cosφ·cosθ, y = r·cosφ·sinθ, z = r·sinφ (plus mount position and yaw).
2. **Ego-motion compensation of the Doppler.** The radar moves with velocity v_s = v_ego + ω × r_mount. A static world point therefore returns ṙ = −v_s·u (u = line of sight). The compensated value **ṙ_c = ṙ + v_s·u** is ≈ 0 for the static world and equals the object's own speed along the line of sight otherwise.
3. **Ghost filter:** points more than 0.5 m below the road are multipath reflections via the road surface and are dropped.
4. **Clustering:** DBSCAN on (x, y), with the extra condition that neighbours agree in compensated Doppler (|Δṙ_c| ≤ 1 m/s), so a moving car next to a guardrail stays separate. The neighbourhood radius **grows with range**, ε = max(2 m, 2.5·r·σ_θ): at 190 m, angular noise alone spreads one object's returns by about 1.7 m per point. A fixed 2 m radius discarded a truck at 190 m during testing.
5. **Per cluster:** centroid, mean raw and compensated Doppler, Doppler spread (micro-Doppler: swinging legs widen it), height mean and its uncertainty σ_z = r·σ_φ/√n.
6. **Elevation gating of stationary clusters** (the classic radar problem: a stationary return could be a stopped truck, a bridge or a manhole cover). The 2σ height interval is compared with the obstacle band 0.3–4.0 m above the road:
   - entirely inside → **height confirmed** (a solid road obstacle);
   - entirely outside → **rejected** (gantry, bridge, plate in the road);
   - overlapping → **unconfirmed** (kept, but it can only raise CAUTION).

   Moving clusters are never road furniture and are always kept.
7. **Field-of-view edge:** clusters within 10° of the FOV edge are dropped (a cut-off cluster's centroid slides toward the FOV centre). Doppler is not used below 10 m: there an extended object spans a wide angle, so its mean Doppler no longer equals the Doppler at its centroid. In testing, both effects made parked cars appear to slide sideways toward the ego at 1.3 m/s.

### 4.5 Radar fusion
- Radar objects are matched (Hungarian, gate 2.5 m + 2 % of range) to the camera/LiDAR detections. The distance is measured to the closer of the LiDAR box centre and its **near face**, because radar returns come mostly from the face toward the sensor, and a truck's centre is 4.5 m behind it.
- **Matched:** the LiDAR position is kept (more precise) and the radar's **range rate** is attached.
- **Unmatched but inside an already-detected object:** absorbed (long vehicles split into several radar clusters).
- **Radar-only:** a new `unknown_obstacle` detection with radar position uncertainty (range σ ≈ 0.5 m, azimuth σ ≈ r·σ_θ/√n + 0.4 m) and its Doppler. This is how objects between 100 and 200 m enter the tracker.

### 4.6 Multi-object tracking
**State** x = [p_x, p_y, v_x, v_y, a_x, a_y]ᵀ in the world frame. **Constant-acceleration model** per axis:

```
F = [1 dt dt²/2; 0 1 dt; 0 0 1]      Q = σ_j² · [dt⁵/20 dt⁴/8 dt³/6; dt⁴/8 dt³/3 dt²/2; dt³/6 dt²/2 dt]
```

σ_j (white jerk) per class: vehicle 2.5, pedestrian/cyclist 3.0, animal 2.0, unknown 2.5 m/s³. Updates use the Joseph form for numerical stability.

- **Position update:** linear Kalman update with the detection's 2×2 covariance.
- **Doppler update (EKF):** h(x) = (v − v_s)·u with u = (p − s)/|p − s|; Jacobian ∂h/∂p = ((v − v_s) − h·u)/|p − s|, ∂h/∂v = u. Grossly inconsistent Doppler values (wrong association) are skipped. In a unit test, 4 frames with Doppler cut the velocity uncertainty of a stationary object by more than 70 % compared with position-only updates.
- **Gating:** Mahalanobis d² < 13.8 (χ², 2 dof, 99.9 %); class groups (vehicle / vulnerable road user / animal) never associate with each other; unknown associates with anything.
- **Assignment:** Hungarian (optimal), cost = d².
- **Initialisation:** position from the first detection; velocity from two consecutive detections when that estimate is precise (σ < 4 m/s), otherwise from the filter.
- **Track management:** confirmed after 3 hits; tentative tracks are deleted after 3 consecutive misses, confirmed tracks after 11 (≈ 0.5 s).
- **Duplicate merging:** two tracks that are statistically at the same place (d² < 4 or within half an object length) with similar velocity are merged into the older one. Without this, a truck split into two radar clusters produced two tracks and broke the radar → LiDAR hand-over at 100 m. Two precise objects 1.2 m apart (two pedestrians) correctly stay separate.
- **Class:** confidence-weighted votes. A radar-only track at 150 m is `unknown_obstacle` and becomes `truck` once the camera recognises it.
- **Height confirmation:** a track counts as a solid road obstacle after **2 LiDAR/camera detections** or **3 height-confirmed radar detections**. A single random false LiDAR detection landing on a radar-only manhole track was enough to cause a false alarm before this rule.
- **Maturity** (required for WARNING/CRITICAL): confirmed; height confirmed; measured in the last 2 frames (not coasting); velocity well known. With Doppler, "well known" means radial σ < 1.5 m/s and tangential σ < 3 m/s; without it, σ < 1.5 m/s in both directions.
- **Doppler-stationary model** (a simplified IMM "stationary" mode): after 5 consecutive Doppler updates with world radial velocity < 0.6 m/s and no significant sideways motion, a non-pedestrian, non-animal track is treated as stationary. That removes the long-range azimuth noise from its velocity uncertainty. Pedestrians and animals are excluded because a crossing pedestrian or cow also shows zero Doppler.

### 4.7 Behaviour and trajectory prediction
Horizon 5 s, step 0.1 s. The key rule is: **only extrapolate motion that is statistically significant.** A filtered acceleration that is pure noise, extrapolated for 3 s, moves an object by metres. In testing, that alone produced most of the false alarms.

- **Stationary test:** vᵀP_v⁻¹v < 9.21 (not distinguishable from 0 at 99 %) or |v| < 0.5 m/s → the object is predicted as stationary.
- **Vehicles:** lateral velocity (relative to the road direction, taken as the ego heading in V1) is kept only if it exceeds 3σ; only the **longitudinal** acceleration is extrapolated, and only if it exceeds 1 m/s².
- **Pedestrians and animals:** constant velocity; sideways motion is kept if it exceeds 2σ (more cautious than for vehicles).
- **Stop clamp:** a braking vehicle stops; it never reverses.
- **Covariance** is propagated with the linear model (P ← FPFᵀ + Q).
- **Behaviour cues:** hard braking (longitudinal a < −3 m/s²), cut-in (lateral speed toward the ego > 0.5 m/s), crossing (pedestrian/animal moving toward the ego path > 0.3 m/s).

**Ego prediction:** CTRV (constant turn rate and velocity), plus "what-if" braking rollouts: keep the current motion for a delay, then brake at a given deceleration along the same path.

### 4.8 Collision risk
1. **Footprint conflict:** ego and object are oriented rectangles (plus a 0.3 m margin each); overlap is tested with the separating axis theorem at every prediction step. The first overlap time is the **predicted TTC**.
2. **Classic TTC** for an in-lane lead object: smallest positive root of gap + v_rel·t + ½·a_rel·t² = 0.
3. **Collision probability:** 200 Monte Carlo samples of position and velocity from the track covariance around the significance-gated mean. Each sample is rolled out and checked against the ego path. Acceleration noise is not sampled; doing so held a parked truck's probability at 0.2–0.4. For Doppler-stationary tracks the velocity spread is 0.3 m/s.
4. **Warning level from what-if braking rollouts:**

| Level | Condition (with mature track for WARNING/CRITICAL) |
|---|---|
| CRITICAL | system braking after 0.3 s at 9 m/s² still collides, and P ≥ 0.7 |
| WARNING | a driver reacting after 1.2 s and braking at 6 m/s² still collides, and P ≥ 0.5 |
| CAUTION | a driver reacting after 2.2 s still collides, and P ≥ 0.2 |

For a constant closing speed v this reduces to the textbook rule **TTC ≤ t_react + v / (2·a_driver)**. Unlike a fixed TTC threshold, it also handles braking lead vehicles and crossing pedestrians correctly, because it asks the physical question directly: *can a normal driver still avoid this if warned now?*

### 4.9 Warning manager
A level must persist for 3 frames (0.15 s) to be raised (CRITICAL: 1 frame), and must be absent for 10 frames (0.5 s) to be lowered. The 3-frame rule removed rare two-frame false alarms at a cost of 0.05 s of lead time.

### 4.10 Design calculation: how far must Part 1 see?
To warn a normal driver early enough, the obstacle must be confirmed and mature before the gap shrinks to the driver's stopping distance:

d_required ≈ v·t_react + v²/(2·a_driver) + v·t_track + ½(L_ego + L_obj)

with t_react = 1.2 s, a_driver = 6 m/s², t_track ≈ 0.5 s (confirmation + velocity convergence), lengths ≈ 7 m:

| Ego speed | Stopping distance | **Required detection range** | Covered by |
|---|---|---|---|
| 40 km/h | 24 m | ~36 m | camera + LiDAR |
| 60 km/h | 43 m | ~59 m | camera + LiDAR |
| 80 km/h | 68 m | ~86 m | camera + LiDAR (just) |
| 100 km/h | 98 m | ~119 m | **radar** |
| 120 km/h | 133 m | ~156 m | **radar** (200 m) |

**Measured consequence (S7, 120 km/h, broken-down truck):** camera + LiDAR warns 2.35 s before impact, when it is already too late for a normal driver. With radar: 3.50 s. The ideal for a 1.2 s / 6 m/s² driver is about 4.0 s. The remaining gap comes from **elevation resolution**: with 0.5° accuracy, radar can only prove that a stationary return at road level is not a manhole cover or a bridge from about 110 m. A sharper elevation channel (better imaging radar) or earlier camera confirmation would close it. This is a quantitative research finding for the report.

---

## 5. Simulation

### 5.1 Two levels of simulation
1. **Synthetic scenario simulator (C++, no CARLA):** exact ground truth, controllable noise, radar model with static guardrail clutter, multipath ghosts, micro-Doppler, overhead gantries and road plates, about 10 ms per scenario. Used for algorithm development and as an automated regression test.
2. **CARLA 0.9.16:** realistic rendering, LiDAR and radar ray casting, traffic and pedestrians. Used for V1 evaluation and the demo video.

### 5.2 The "inattentive driver" concept
In every scenario the ego keeps driving and **does not react** (CARLA Traffic Manager with `ignore_vehicles_percentage = 100` and `ignore_walkers_percentage = 100`). The collision therefore really happens, and its time is the ground truth for **warning lead time = t_collision − t_first_WARNING**. This is the central V1 metric.

### 5.3 CARLA setup (Windows, RTX 5050 laptop)
1. Download the CARLA 0.9.16 Windows package and unzip it (no Unreal build needed).
2. Start the server: `CarlaUE4.exe -quality-level=Low` (Low keeps the frame rate up on an 8 GB GPU; use Epic only for demo footage).
3. Install the matching Python client (section 3.1).
4. `python simulation/carla/record_run.py --scenario lead_brake` (Town04: long highways).
5. **Axis check (once):** `python simulation/carla/view_run.py <run> --frame 100`. Vehicles to the left must appear at +y, and LiDAR points (black) and radar points (purple) must sit on the green ground-truth boxes.
6. **Radar velocity sign (once):** `python simulation/carla/view_run.py <run> --radar-check`. CARLA does not document the sign of its radar velocity; the check uses the static world (most returns) to tell you whether `radar_velocity_sign` must be +1 or −1. Put the result into `RADAR_VELOCITY_SIGN` in `record_run.py`.

### 5.4 Sensor configuration

| Sensor | Setting |
|---|---|
| RGB camera | 1280×720, FOV 90°, at (1.5, 0, 1.6) m |
| LiDAR | 32 channels, 100 m, 640 k points/s, 20 Hz rotation (one sweep per tick), FOV +10°/−30°, at (0, 0, 2.4) m |
| Radar | `sensor.other.radar`, 200 m, 90° × 20° FOV, 12 k points/s, at (2.4, 0, 0.5) m |
| Collision sensor | ends the run and records the impact time |
| World | synchronous mode, fixed Δt = 0.05 s, ClearNoon weather |

CARLA's radar is idealised (no noise, ghosts or resolution limits). Results on CARLA radar data are therefore optimistic. The synthetic model and the noise/ghost post-processing planned for Part 2 address this.

### 5.5 Scenario catalogue

| ID | Scenario | Synthetic | CARLA recorder |
|---|---|---|---|
| S1 | Lead vehicle brakes hard (6 m/s²) | ✅ | `lead_brake` |
| S2 | Broken-down truck in lane, 90 km/h | ✅ | `stationary_vehicle` |
| S3 | Much slower vehicle ahead | ✅ | (free_drive + traffic) |
| S4 | Cut-in from the adjacent lane | ✅ | V1 stretch |
| S5 | Pedestrian emerging from behind a parked vehicle | ✅ | `pedestrian_cross` |
| S6 | Animal crossing a highway | ✅ | real dashcam video (CARLA has no animals) |
| S7 | Broken-down truck at **120 km/h** (beyond LiDAR range) | ✅ | `stationary_vehicle --speed 120` |
| F1–F5 | Overtaking car, pedestrian on footpath, steady following, parked cars, oncoming traffic | ✅ | `free_drive` |
| F6 | Overhead sign gantry (radar sees it) | ✅ | find a gantry/bridge in Town04 |
| F7 | Manhole cover / metal plate in the lane (radar sees it) | ✅ | – (CARLA has no such prop by default) |

### 5.6 Building your own dataset

Your dataset is a set of CARLA recordings whose conditions you control and record. Everything that defines a run is a `record_run.py` option and is saved in the run's `run_config.json`:

| Category | Options |
|---|---|
| Scenario | `--scenario` (lead_brake, stationary_vehicle, pedestrian_cross, free_drive), `--speed`, `--duration`, `--seed`, `--town` |
| Weather | `--weather` preset: clear, cloudy, rain, heavy_rain, fog, dense_fog, sunset_glare, night, night_rain, night_fog (`--list-weather` shows the values). Any field can be overridden: `--fog-density`, `--fog-distance`, `--precipitation`, `--wetness`, `--cloudiness`, `--sun-altitude-angle` (< 0 = night), `--sun-azimuth-angle`, `--wind-intensity` |
| Traffic | `--traffic` none / light (20 vehicles, 20 pedestrians) / medium (60, 50) / heavy (120, 100); `--vehicles`, `--walkers`, `--traffic-speed-diff` (% vs speed limit) |
| Unexpected events | `--surprise-brakes N`: on average N times per minute a random vehicle 15–60 m ahead in the ego lane brakes hard for 2–4 s; `--jaywalk P`: pedestrians cross outside crossings with probability P. Every event is logged in `events.csv` |
| Storage | `--image-format jpg` (default, ~5–8× smaller) or `png` |

**Campaigns.** `simulation/carla/generate_dataset.py` records a whole parameter matrix from a YAML file, resumes after interruptions, then runs YOLO → `part1_replay` (full, `no_radar` and `lidar_only` configurations) → `evaluate_run.py` on every run. `experiments/summarize_dataset.py` turns it into one table:

```bash
python simulation/carla/generate_dataset.py --config experiments/configs/v1_campaign.yaml --dry-run   # cost
python simulation/carla/generate_dataset.py --config experiments/configs/v1_campaign.yaml             # record
python simulation/carla/generate_dataset.py --config experiments/configs/v1_campaign.yaml --process-only
python experiments/summarize_dataset.py datasets/runs/v1_campaign                                     # table
```

| Config | Runs | Simulated | Disk (max, JPG) | Purpose |
|---|---|---|---|---|
| `quick_trial.yaml` | 3 | 0.8 min | 0.6 GB | first check that everything works |
| `v1_campaign.yaml` | 56 | 27 min | ~22 GB | the V1 results (4 scenarios × speeds × 5 seeds + free driving) |
| `weather_and_surprises.yaml` | 51 | 28 min | ~23 GB | weather / lighting robustness and unexpected events |
| `synthetic_test.yaml` | 6 | 1 min | <1 GB | tool-chain test without CARLA (`--backend synthetic`) |

Disk is dominated by LiDAR (~0.5 MB per frame); runs that end in a collision are shorter, so real usage is lower. Measure your recording rate on the quick trial and pass it as `--sim-fps` for realistic time estimates.

**Tested here:** the generator, processing chain and summary were run end to end with the synthetic backend (6 runs × 3 sensor configurations; at 120 km/h radar gave 3.60 s lead vs 2.30 s without). The CARLA backend has not been run against a live server yet.

---

## 6. Testing

### 6.1 Test pyramid

| Level | What | How | Status |
|---|---|---|---|
| Unit | Hungarian, Kalman convergence, ID keeping, clutter rejection, braking stop, TTC formula, SAT overlap, braking-rollout distance (82.08 m at 25 m/s), warning boundary, hysteresis, RANSAC + clustering + gantry rejection, projection, mono range, fusion, frame transforms, **radar Doppler compensation + ghosts, radar elevation gating (truck kept / gantry and plate rejected), EKF Doppler update, radar fusion (match / absorb / radar-only), duplicate merging** | `part1_tests` (CTest) | ✅ all pass |
| Scenario acceptance | 14 synthetic scenarios with noise, clutter and radar | `part1_tests`, `part1_scenarios --seeds 50` | ✅ 700/700 |
| Sensor ablation | same scenarios without radar | `part1_scenarios --no-radar` | ✅ run (table below) |
| Tool chain | synthetic run folder (camera boxes, LiDAR, radar) → `part1_replay` → `evaluate_run.py` → `view_run.py` | scripts | ✅ |
| CARLA | recorded scenarios, ≥ 10 runs each | recorder + replay + evaluate | ⏳ next |
| Real video | YOLO on Indian dashcam footage (animals, auto-rickshaws) | `yolo_detect.py --video` | ⏳ |

### 6.2 V1 acceptance criteria

| Metric | Target |
|---|---|
| Warning lead time, S1/S2/S3/S6 | ≥ 1.5 s in every run |
| Warning lead time, S4 (cut-in), S5 (emerging pedestrian) | ≥ 1.0 s in every run |
| Warning lead time, S7 (120 km/h) | ≥ 2.8 s in every run (sensor-limited, see 4.10) |
| False alarms (F1–F7) | 0 frames at WARNING or above |
| TTC error (last 5 s before impact) | ≤ 0.3 s mean absolute |
| Tracking recall (GT actors ahead, ≤ 80 m) | ≥ 90 % |
| C++ processing time | ≤ 10 ms per frame |

### 6.3 Current results: synthetic simulator, 50 noise seeds per scenario

| Scenario | Camera + LiDAR + radar | Camera + LiDAR only (ablation) |
|---|---|---|
| | passed · lead min / median · TTC error | passed · lead min / median · TTC error |
| S1 lead vehicle hard braking | 50/50 · **2.90 / 2.95 s** · 0.07 s | 50/50 · 2.30 / 2.50 s · 0.19 s |
| S2 stationary truck, 90 km/h | 50/50 · 3.20 / 3.20 s · 0.06 s | 50/50 · 3.15 / 3.20 s · 0.03 s |
| S3 slow vehicle ahead | 50/50 · 2.50 / 2.55 s · 0.03 s | 50/50 · 2.15 / 2.70 s · 0.16 s |
| S4 cut-in | 50/50 · 1.55 / 1.60 s · 0.08 s | 50/50 · 1.30 / 1.65 s · 0.15 s |
| S5 pedestrian emerging | 50/50 · 1.00 / 1.40 s · 0.03 s | 50/50 · 1.00 / 1.50 s · 0.04 s |
| S6 animal crossing | 50/50 · 2.10 / 2.75 s · 0.03 s | 50/50 · 2.05 / 2.60 s · 0.04 s |
| S7 stationary truck, **120 km/h** | 50/50 · **3.50 / 3.50 s** · 0.07 s | **0/50** · 2.25 / 2.35 s |
| F1–F7 false-alarm tests | 50/50 each · 0 warning frames | 50/50 each · 0 warning frames |

**What radar adds:**
- **Sudden braking:** about 0.6 s more warning in the worst case. Doppler measures the closing speed directly, which also cuts the TTC error from 0.19 s to 0.07 s.
- **Cut-ins:** a better worst case.
- **High speed:** S7 is impossible without radar.
- **No help for crossing pedestrians (S5):** Doppler measures motion *along* the line of sight, and a crossing pedestrian moves *across* it. This is expected physics, not a defect.

**Replay test** (synthetic run folders through the full replay chain):

| Run | With radar | Without radar |
|---|---|---|
| 72 km/h toward a truck 120 m ahead, parked car at the roadside, heading 30° | 2.80 s lead, no false alarm, recall 98 % | 2.80 s |
| 120 km/h toward a truck 230 m ahead | **3.65 s** lead (truck tracked by radar from ~160 m) | 2.35 s |

C++ processing time: 2.0–2.3 ms per frame including radar.

S5 remains the hardest case: the pedestrian is hidden until 2.3 s before impact, so the minimum lead time sits exactly at the 1.0 s requirement. This motivates the event camera (V3) and occlusion-aware caution near parked vehicles.

### 6.4 What has not been verified yet
- `record_run.py` has not been run against a live CARLA server. The first run needs the axis check and the radar sign check (section 5.3).
- `yolo_detect.py` has not been run here (no GPU); its YOLO26 timing must be measured on the laptop.
- The C++ code has been built and tested with GCC on Linux. The first MSVC build on Windows may raise warnings that need small fixes.
- All results so far are synthetic. The radar model in particular is simplified (returns spread over the object's footprint, independent per-point noise), and CARLA's radar is idealised. Real 4D radar data (View-of-Delft, K-Radar) would be the real test of the radar pipeline.

---

## 7. Known limitations and planned improvements

| Limitation | Impact | Improvement |
|---|---|---|
| Single-model CA filter + simple Doppler-stationary rule | trades smoothness against manoeuvre response | full **IMM** (stationary / CV / CA) in V2 |
| Road direction = ego heading | lateral gating is approximate on curves and at junctions | lane geometry from the CARLA map / lane detection |
| LiDAR box only covers visible faces | object length underestimated head-on | class-based size prior from fusion |
| Ego pose from simulator ground truth | real cars need odometry | IMU + wheel-speed EKF (V2) |
| Front-only sensing | coasting tracks beside the car can only raise CAUTION | Part 2: 360° radar and blind-spot coverage |
| Radar elevation accuracy (0.5°) | stationary objects confirmed only from ~110 m → S7 lead 3.5 s vs 4.0 s ideal | camera-confirmed height at long range; better elevation resolution |
| Radar near field / FOV edge | Doppler unreliable below 10 m and near ±45° | corner radars (Part 2) |
| Idealised CARLA radar | optimistic results on CARLA data | noise / ghost / resolution post-processing (Part 2) |
| Late, box-level fusion | loses information in heavy occlusion | BEV / mid-level fusion research (V3) |
| Occluded pedestrians | S5 lead time is the minimum (1.0 s) | occlusion-aware caution, event camera (V3) |

---

## 8. Milestones to V1 (target: 2nd week of January 2027)

| Week | Dates | Deliverable |
|---|---|---|
| 1 | 9–15 Nov | Build on Windows (MSVC), all tests green (700/700); CARLA 0.9.16 running; synthetic run → replay → evaluate on the laptop |
| 2 | 16–22 Nov | `quick_trial.yaml` campaign; axis check; **radar sign check**; YOLO on recorded frames; first replays; measure recording rate |
| 3 | 23–29 Nov | Tune LiDAR and radar processing on CARLA data (ground, height band, cluster sizes, radar point density); tracking recall ≥ 90 % |
| 4 | 30 Nov–6 Dec | **`v1_campaign.yaml`** (56 runs) recorded, processed and summarised; radar ablation on CARLA data |
| 5 | 7–13 Dec | Cut-in scenario in CARLA; YOLO on Indian dashcam video (animals, auto-rickshaws); optional `weather_and_surprises.yaml` |
| 6 | 14–20 Dec | Live mode for the demo (pybind11 or socket), on-screen warning overlay |
| 7 | 21–27 Dec | Evaluation campaign; plots; compare against a detection-only baseline (distance threshold) |
| 8 | 28 Dec–3 Jan | Demo video, README results, documentation |
| 9 | 4–10 Jan | Buffer, fixes, **V1 release tag** |

---

## 9. References
- Zhang, X. et al. (2017). *Efficient L-Shape Fitting for Vehicle Detection Using Laser Scanners.* IEEE Intelligent Vehicles Symposium.
- Bar-Shalom, Y., Li, X. R., Kirubarajan, T. (2001). *Estimation with Applications to Tracking and Navigation.* Wiley. (CA model, gating, EKF, IMM)
- Ester, M. et al. (1996). *A Density-Based Algorithm for Discovering Clusters in Large Spatial Databases with Noise (DBSCAN).* KDD.
- Kuhn, H. W. (1955). *The Hungarian Method for the Assignment Problem.*
- CARLA Simulator documentation, release 0.9.16.
- Ultralytics YOLO26 documentation.
