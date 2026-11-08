# IPVSEAS
### Intelligent Predictive Vehicle Safety and Emergency Assistance System

**A human-centred, privacy-preserving safety layer that predicts road hazards, watches the traffic the driver cannot see, checks whether the driver can still respond, and steps in only when it has to.**

> **Design philosophy:** The driver drives. IPVSEAS watches, predicts and warns, and it intervenes only when a collision is imminent or the driver is unable to respond. It never watches the driver's face.

---

## 1. Why this project exists

Most road accidents are not caused by hazards that are impossible to detect. They are caused by hazards that are detected **too late**: a broken-down truck with no hazard lights in winter fog, a car that cuts into your lane, a pedestrian stepping out from behind a parked bus, cattle standing on a highway at night, a vehicle in your blind spot, or a driver who has suddenly become unconscious.

Full self-driving tries to solve this by removing the driver. IPVSEAS takes the opposite position: keep the driving experience fully human, and add an intelligent safety layer that

1. **predicts** dangerous situations before they become unavoidable,
2. **keeps seeing** when the human cannot (fog, smog, rain, night, glare, blind spots),
3. **knows whether the driver is still responding**, without cameras on their face, and
4. **acts** through a graded escalation, from a gentle haptic nudge up to automatic braking, evasive steering or a minimum-risk stop, only when needed.

## 2. The problems IPVSEAS solves

| # | Real-world problem | What IPVSEAS does |
|---|---|---|
| P1 | Stopped, broken-down or unusually slow vehicles ahead | Detects them, estimates relative motion, predicts time-to-collision (TTC) and warns early |
| P2 | Sudden braking or lane changes by other vehicles | Tracks every road user and predicts their trajectory, not just their current position |
| P3 | Pedestrians and animals entering the road (including hit-and-run risk at night) | Predicts whether their future path intersects ours |
| P4 | Fog, smog, rain, dust, night: the driver cannot see far | Fuses LiDAR, **4D imaging radar**, camera and event-camera data, and reports **how confident** it is |
| P5 | Blind spots, fast vehicles from behind, cut-ins (a TCAS-like view of the road) | Builds a 360° bird's-eye picture of surrounding traffic and predicts conflicts |
| P6 | Rare, strange hazards: debris, overturned vehicles, wrong-way drivers, unlit tractor-trolleys | Treats the **long tail** as a first-class problem (class-agnostic obstacle detection plus a corner-case scenario library) |
| P7 | Collision becomes imminent and braking alone is not enough | Automatic emergency braking, then **emergency evasive steering** if the adjacent lane is verified clear |
| P8 | Reversing into people or objects; being rear-ended during hard braking | **Rear automatic braking** while reversing, plus rear-aware braking decisions |
| P9 | Driver incapacitation (medical emergency, unconsciousness) | Detects abnormal driver–vehicle interaction (steering, pedals, horn, seat), confirms with haptic and audible prompts, then performs a simulated minimum-risk manoeuvre and emergency notification |

## 3. System overview: three parts and one safety brain

```text
                SENSOR SUITE (outward-facing only)
   Camera · LiDAR · 4D imaging radar · Event camera · Vehicle CAN
            Steering · Pedals · Horn · Seat pressure
                              │
     ┌────────────────────────┼────────────────────────┐
     ▼                        ▼                        ▼
┌───────────────┐   ┌───────────────────┐   ┌──────────────────────┐
│   PART 1      │   │     PART 2        │   │      PART 3          │
│ Road-user &   │   │ Adverse weather & │   │ Driver incapacitation│
│ obstacle      │◄─►│ blind-spot safety │   │ & emergency          │
│ collision     │   │ (confidence, 360° │   │ assistance           │
│ prediction    │   │  traffic, rear)   │   │ (no face camera)     │
└──────┬────────┘   └────────┬──────────┘   └──────────┬───────────┘
       │ hazards, TTC        │ sensor confidence,      │ driver
       │ trajectories        │ free lanes, rear risk   │ responsiveness
       └─────────────────────┼─────────────────────────┘
                             ▼
              ┌──────────────────────────────┐
              │   SAFETY DECISION ENGINE     │
              │  graded escalation ladder    │
              └──────────────┬───────────────┘
                             ▼
   Warn (visual/audio/haptic) → Brake assist → AEB →
   Evasive steering → Minimum-risk manoeuvre → Emergency notification
```

### Part 1: Predictive Road-User and Obstacle Collision Prediction
*What is happening ahead, and what is about to happen?*

- Detection of vehicles, pedestrians, cyclists, **animals** and stationary obstacles
- Multi-object tracking (Kalman / Extended Kalman filter)
- Behaviour prediction: sudden braking, slow vehicles, lane-change intent, pedestrian crossing intent
- Short-term trajectory prediction and path-conflict detection
- TTC and probabilistic collision-risk estimation
- **Long-tail handling:** class-agnostic LiDAR obstacle detection, so that an object the camera cannot name is still treated as an obstacle; uncertainty / out-of-distribution scoring for camera detections
- **Event camera** channel for fast-emerging hazards (a pedestrian stepping out from behind a parked vehicle)

### Part 2: Adverse-Weather Perception and Blind-Spot Safety
*Can the vehicle keep seeing when the driver can't, and in every direction?*

- Fog, smog, rain, dust, night and glare perception
- **Per-sensor confidence estimation:** the system never reports "road clear" just because one sensor went blind
- LiDAR + **4D imaging radar** + camera fusion, with the event camera for high-dynamic-range scenes (headlight glare, tunnel exits)
- 4D radar as the all-weather backbone: range, azimuth, **elevation** and Doppler velocity per point, so stopped vehicles can be told apart from overhead bridges and signs
- 360° bird's-eye-view traffic picture (the TCAS-inspired view)
- Blind-spot monitoring, rear-approach warnings, lane-change and cut-in conflict prediction
- **Rear automatic braking:** stops the vehicle when reversing towards a person or obstacle
- **Rear-aware braking:** tells the decision engine how risky hard braking is when a vehicle is close behind

### Part 3: Driver Incapacitation Detection and Emergency Assistance
*What if the driver can no longer respond?*

- **Privacy by design: no driver-facing camera and no facial data.**
- Driver–vehicle interaction analysis: steering activity and grip, accelerator and brake patterns, abnormal horn activation (for example a head or hand resting on the horn), seat-pressure changes
- Context-aware anomaly detection: no steering on a straight highway is normal; no steering in a curve is not
- Responsiveness verification through haptic (steering wheel and seat) and audible prompts with an acknowledgement input
- Simulated minimum-risk manoeuvre (MRM): slow down, move to a safe stopping place, hazard lights
- Simulated emergency notification (location and vehicle state)

### Safety Decision Engine (the integration layer)

Every part feeds one decision engine that escalates through defined levels:

| Level | Response | Typical trigger |
|---|---|---|
| L0 | Monitor | No significant risk |
| L1 | Information cue | Risk rising (for example a vehicle entering the blind spot) |
| L2 | Warning: visual, audible and haptic | Collision risk above threshold, TTC shrinking |
| L3 | Brake assist / pre-fill | Driver is braking, but not hard enough |
| L4 | Automatic emergency braking (AEB) | Collision imminent, driver not reacting |
| L5 | **Emergency evasive steering** | Braking alone cannot avoid the collision **and** Part 2 confirms the escape lane is free |
| L6 | Minimum-risk manoeuvre + emergency notification | Part 3 confirms the driver is unresponsive |

Clear driver input (firm steering, kick-down, brake) overrides L1–L5. Exactly when an L4/L5 intervention should still yield to the driver is an explicit design question in this project.

## 4. Research tracks beyond the core pipeline

### 4.1 The long-tail problem
Real roads, especially Indian roads, are full of hazards no dataset has enough examples of: cattle, unlit tractor-trolleys, wrong-way two-wheelers, fallen loads, potholes and debris. IPVSEAS addresses this by:
- detecting obstacles by **geometry** (LiDAR / radar) as well as by **class** (camera), so unknown objects still count;
- scoring detector uncertainty, and escalating caution when the scene is out-of-distribution;
- maintaining a **corner-case scenario library** in CARLA, evaluated separately from common scenarios;
- evaluating on corner-case datasets such as CODA and the India Driving Dataset (IDD).

### 4.2 End-to-end neural networks
End-to-end models (raw sensors → risk or driving decisions) are a major trend, but they are hard to verify. IPVSEAS uses them as a **comparison arm, not the safety core**:
- an end-to-end collision-risk predictor trained in CARLA, compared against the modular pipeline on the same scenarios;
- explicit comparison of accuracy, warning lead time, false alarms and long-tail behaviour;
- a rule-based safety supervisor that can always veto a learned output.

The research question: *where does end-to-end learning beat the modular pipeline, and where does it fail silently?*

### 4.3 Event cameras
Event (neuromorphic) cameras report brightness changes per pixel with microsecond timing and very high dynamic range. IPVSEAS investigates them for:
- fast-emerging hazards (pedestrians, animals, cut-ins);
- headlight glare, night driving and tunnel exits, where frame cameras saturate.

CARLA includes a simulated DVS event camera; real-data validation can use public driving event datasets such as DSEC.

### 4.4 Emergency evasive steering
When the relative speed is high, steering around an obstacle needs less distance than braking to a stop. IPVSEAS models the brake-versus-steer decision boundary, checks the escape lane with Part 2, and executes evasive steering in simulation only.

### 4.5 Rear automatic braking
Low-speed reversing protection (people, poles, vehicles behind), and rear-collision awareness for forward AEB decisions.

### 4.6 4D imaging radar
A 4D imaging radar measures **range, azimuth, elevation and radial (Doppler) velocity** for every point, giving a sparse point cloud in which every point also carries a speed. IPVSEAS uses it to:
- **solve the stationary-vehicle problem:** conventional radar has no elevation, so it cannot separate a stopped truck from an overpass, a sign gantry or a manhole cover, and stationary returns are often filtered out. Elevation lets IPVSEAS trust stationary returns at road height;
- act as the **all-weather backbone** in fog, smog and rain, where cameras and LiDAR degrade;
- feed **measured velocity** straight into tracking, for faster reaction to sudden braking;
- separate pedestrians and animals from static clutter using **micro-Doppler** (limb motion);
- improve escape-lane checks for evasive steering and rear coverage for rear automatic braking.

Known limits: much sparser and noisier than LiDAR, multipath "ghost" detections (guardrails, tunnels, trucks), and Doppler gives only the radial velocity component.

The research question: *can camera + 4D radar match camera + LiDAR, and beat it in fog and rain, at a fraction of the sensor cost?*

## 5. Technology stack

| Area | Technologies |
|---|---|
| Simulation | CARLA (+ ScenarioRunner for corner cases) |
| Languages | **C++ (~60%)**: tracking, fusion, prediction, risk, decision engine. **Python (~40%)**: model training, datasets, evaluation, CARLA scripting |
| Computer vision | OpenCV, YOLO (Ultralytics) |
| Point clouds | PCL, Open3D (LiDAR and 4D radar) |
| 3D detection | OpenPCDet (candidate; run under WSL2/Ubuntu) |
| Linear algebra | Eigen |
| Deep learning | PyTorch → ONNX Runtime for C++ inference |
| Tracking / fusion | Kalman filter, EKF, probabilistic data association |
| Build | CMake + vcpkg |
| Version control | Git and GitHub |

## 6. Development roadmap

| Version | Scope | Target |
|---|---|---|
| **V1: Core prototype** | Part 1 in clear weather: camera + LiDAR detection, tracking, TTC, trajectory prediction, forward-collision warnings for vehicles and pedestrians in CARLA, plus a recorded demo | 2nd week of January 2027 |
| **V2: Extended prototype** | 4D imaging radar pipeline and fusion, weather degradation and sensor confidence, blind spot, cut-in prediction, rear automatic braking, AEB, evasive steering, corner-case scenario library | After V1 |
| **V3: Research prototype** | Part 3 driver responsiveness + haptics + MRM, event-camera channel, end-to-end comparison arm, full integration and evaluation, technical report | After V2 |

Features are added in this order on purpose: a working V1 matters more than six half-built modules.

## 7. Evaluation

| Area | Metrics |
|---|---|
| Detection | Precision, recall, mAP, missed-detection rate, with a **separate long-tail split** |
| Tracking | MOTA, IDF1, ID switches |
| Prediction | ADE / FDE trajectory error |
| Collision warning | TTC error, warning lead time, false-alarm rate |
| Weather robustness | Performance drop per weather condition, confidence calibration |
| Interventions | Collisions avoided, AEB vs evasive-steering success, unnecessary interventions |
| Driver monitoring | Incapacitation detection precision/recall, false-alarm rate, time-to-detect |
| System | End-to-end latency, FPS, GPU/CPU use |

Key comparisons: detection-only vs prediction-based warnings · single sensor vs fusion · camera + 4D radar vs camera + LiDAR · conventional vs 4D radar on stationary vehicles · fixed-threshold vs context-aware driver monitoring · modular vs end-to-end · frame camera vs event camera.

## 8. Scope and safety

IPVSEAS is a **simulation-based research prototype**. It does not connect to or control any real vehicle's steering, braking or emergency-calling systems. Real-world intervention functions require certified hardware, functional-safety engineering (ISO 26262 / SOTIF) and regulatory approval.

## 9. Repository structure

```text
IPVSEAS/
├── docs/                              Problem statement, architecture, methodology
├── simulation/carla/                  CARLA setup, scenarios, weather models
├── part_1_road_perception/
│   ├── camera/                        Frame-camera pipeline (video_capture, detection)
│   ├── lidar/                         Point-cloud processing, class-agnostic obstacles
│   ├── radar/                         4D imaging radar: range, azimuth, elevation, Doppler
│   ├── event_camera/                  DVS event processing
│   ├── sensor_fusion/                 Calibration, association, fused objects
│   ├── tracking/                      KF / EKF multi-object tracking
│   ├── behaviour_prediction/          Braking, lane-change and crossing intent
│   ├── long_tail/                     OOD scoring, corner-case evaluation
│   └── collision_prediction/          TTC, path conflict, risk score
├── part_2_weather_and_blindspot/
│   ├── weather_perception/            Visibility estimation, degradation models
│   ├── sensor_confidence/             Per-sensor reliability
│   ├── blindspot_detection/
│   ├── surrounding_traffic/           360° BEV traffic picture
│   └── rear_automatic_braking/        Reverse RAB, rear-aware braking
├── part_3_emergency_assistance/
│   ├── driver_interaction/            Steering, pedal, horn, seat signals
│   ├── incapacitation_detection/      Context-aware anomaly detection
│   ├── responsiveness_check/          Haptic / audible prompts, acknowledgement
│   ├── minimum_risk_manoeuvre/
│   └── emergency_notification/
├── safety_decision_engine/
│   ├── escalation/                    L0–L6 escalation logic
│   ├── warnings_hmi/                  Visual, audible, haptic patterns
│   ├── aeb/                           Automatic emergency braking
│   └── evasive_steering/              Brake-vs-steer decision, escape-lane check
├── end_to_end/                        End-to-end comparison arm
├── integration/                       Full-system runner
├── datasets/  experiments/  results/
├── src/cpp/                           C++ entry point
├── CMakeLists.txt  CMakePresets.json  vcpkg.json
├── requirements.txt
└── README.md
```

## 10. Building

**Python**
```bash
python -m venv ipvseas_env
ipvseas_env\Scripts\activate        # Windows
pip install -r requirements.txt     # install PyTorch separately with the CUDA wheel
```

**C++ (CMake)**
```bash
# Option A: vcpkg (recommended for anyone cloning the repo)
cmake --preset vcpkg
cmake --build --preset vcpkg

# Option B: locally installed OpenCV / Eigen
# copy CMakeUserPresets.json.example to CMakeUserPresets.json, set your paths, then:
cmake --preset local
cmake --build --preset local
```

## 11. Status

**Current stage:** Development environment ready (MSVC, CMake, OpenCV, Eigen). Camera video pipeline started. Next: YOLO detection on recorded road video, then CARLA setup for V1.

---

**IPVSEAS: Perceive. Predict. Protect.**
