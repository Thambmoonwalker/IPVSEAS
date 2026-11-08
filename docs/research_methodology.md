# IPVSEAS: Research Methodology

## 1. Approach

IPVSEAS is evaluated experimentally in simulation (CARLA), using simulator ground truth as reference, and validated where possible on public real-world datasets. Every part is evaluated **on its own first**, then as part of the integrated system.

The goal is not to show that each algorithm is new. The goal is to measure what the **integrated, predictive, privacy-preserving** approach improves over simpler baselines.

## 2. Experimental Environment

- **CARLA** for roads, traffic, pedestrians, weather and lighting; sensors: RGB camera, LiDAR, radar (used to emulate 4D imaging radar, see below), DVS event camera, IMU/GNSS; vehicle signals.
- **ScenarioRunner** (or custom Python scripts) for repeatable hazard scenarios.
- **Seeds and configs** stored in `experiments/` so every result is reproducible.

### Important limitation: weather in simulation
CARLA's weather mainly changes the rendered camera image; simulated LiDAR and radar are only weakly affected. Without correction, experiments would unfairly favour LiDAR and radar. IPVSEAS therefore:
1. applies **physics-based degradation** to LiDAR point clouds (fog attenuation and backscatter, rain noise, range drop-off; for example following published fog-simulation methods) and to radar (noise and clutter);
2. validates weather results on **real adverse-weather data** (for example the DENSE / Seeing Through Fog dataset).

### Emulating 4D imaging radar in CARLA
CARLA's radar already reports depth, azimuth, altitude (elevation) and velocity per detection, but it is sparse and idealised. IPVSEAS raises its point rate and field of view to approximate an imaging radar, then adds the effects CARLA does not model: range/angle/velocity noise, angular resolution limits, RCS-dependent detection probability, false alarms and multipath ghosts (guardrails, tunnels, large vehicles). Results are validated on real 4D-radar datasets (View-of-Delft, TJ4DRadSet, K-Radar; K-Radar includes fog, rain and snow).

## 3. Scenario Library

### 3.1 Core scenarios
| ID | Scenario | Parts |
|---|---|---|
| S1 | Lead vehicle brakes suddenly | 1 |
| S2 | Stationary broken-down truck, no hazard lights | 1, 2 |
| S3 | Slow vehicle on highway | 1 |
| S4 | Adjacent vehicle cuts into our lane | 1, 2 |
| S5 | Pedestrian crosses from behind a parked vehicle | 1 (+ event camera) |
| S6 | Animal on the road at night | 1, 2 |
| S7 | Vehicle in the blind spot during our lane change | 2 |
| S8 | Fast vehicle approaching from behind while we brake | 2, decision engine |
| S9 | Reversing toward a pedestrian / pole | 2 (rear automatic braking) |
| S10 | Imminent collision where braking alone fails; escape lane free | decision engine (evasive steering) |
| S11 | Same as S10, but escape lane occupied | decision engine (must not steer) |
| S12 | Driver incapacitation on a highway (horn held, no pedal input) | 3 |
| S13 | Driver incapacitation while the vehicle is stopped | 3 |
| S14 | False-alarm traps: intentional horn, hands briefly off wheel on a straight road, rough road | 3 |
| S15 | Stopped truck just before an overpass / sign gantry; manhole cover on an empty road (stationary-object traps) | 1, 2 (4D radar) |

### 3.2 Long-tail (corner-case) scenarios
Fallen cargo · overturned vehicle · wrong-way vehicle · unlit tractor-trolley at night · cattle herd · pothole / debris · unusual vehicles (auto-rickshaw, handcart). Results are reported **separately** from core scenarios, so that good average numbers cannot hide long-tail failures. CARLA has no animal actors by default; animals and debris need imported assets or real-data validation.

### 3.3 Environmental conditions
Clear day · night · fog (light / dense) · rain (light / heavy) · fog + rain · night + fog · glare (low sun, oncoming headlights).

## 4. Experiments

### E-A: Predictive vs reactive warnings (Part 1)
Compare (a) distance-threshold warning, (b) TTC warning, (c) trajectory-prediction warning.
*Measure:* warning lead time, false-alarm rate, missed hazards.

### E-B: Sensor configurations under weather (Part 2)
Camera · LiDAR · 4D radar · LiDAR+4D radar · LiDAR+camera · **camera+4D radar** · LiDAR+4D radar+camera, each **with and without confidence-weighted fusion**.
The key comparison is **camera + 4D radar vs camera + LiDAR** across weather conditions.
*Measure:* detection recall/precision per weather condition, "false clear road" events, confidence calibration.
*Note:* this is a supporting experiment inside Part 2, not the headline of the project.

### E-C: Long-tail robustness
Camera-class-only obstacle detection vs camera + class-agnostic LiDAR obstacles vs + OOD scoring.
*Measure:* recall on the long-tail split, false-alarm rate.

### E-D: Event camera
Frame camera vs frame + event camera on S5, S6 and glare scenes.
*Measure:* detection latency (ms from object appearance to detection), recall in glare.
*Note:* CARLA emulates events from rendered frames, so the simulator step must be short (high tick rate) for latency results to mean anything. Validate on a real event dataset (for example DSEC).

### E-E: Interventions
AEB only vs AEB + evasive steering vs AEB + evasive steering + rear-aware braking, on S8, S10 and S11.
*Measure:* collisions avoided, impact speed when unavoidable, unsafe steering events (steering into occupied lanes), rear-end collisions caused.

### E-F: Driver incapacitation (Part 3)
Fixed timeout rule ("no input for N s") vs multi-signal rules vs context-aware learned anomaly model.
*Data:* simulated driver-interaction traces for normal driving, incapacitation and false-alarm traps (S12–S14); optionally a small set recorded with a game steering wheel and pedals.
*Measure:* precision, recall, false alarms per hour of normal driving, time-to-detect.

### E-G: Haptic warning patterns
Different haptic and audible patterns in a driving-simulator setup (game wheel + vibration motors).
*Measure:* reaction time, correct identification of the warning type.
*Note:* involves human participants; obtain informed consent and follow your institution's ethics requirements.

### E-H: Modular vs end-to-end
Modular pipeline vs end-to-end risk predictor on identical scenarios.
*Measure:* all Part 1 metrics, plus long-tail split behaviour and failure analysis (where does each one fail, and does it fail silently?).

### E-J: Conventional vs 4D imaging radar
Radar with elevation discarded (conventional) vs full 4D radar, on S2 and S15, in clear weather and dense fog.
*Measure:* stationary-vehicle detection rate, false alarms from overhead structures and road plates, warning lead time.

### E-I: Integrated system
All parts running together through the Safety Decision Engine, on combined scenarios (for example S2 in dense fog with an incapacitated driver).
*Measure:* end-to-end latency, correct escalation level, unnecessary interventions.

## 5. Metrics

| Area | Metrics |
|---|---|
| Detection | Precision, recall, F1, mAP, missed-detection rate (core and long-tail splits) |
| Tracking | MOTA, IDF1, ID switches |
| Prediction | ADE, FDE |
| Risk | TTC error, warning lead time, false-alarm rate, collision-probability calibration |
| Weather | Performance drop vs clear weather; "false clear" count; confidence calibration |
| Intervention | Collision avoidance rate, residual impact speed, unsafe-manoeuvre count |
| Driver monitoring | Precision, recall, false alarms per hour, time-to-detect |
| System | Latency (sensor → action), FPS, CPU/GPU use |

## 6. Datasets

| Dataset | Use |
|---|---|
| CARLA-generated | Main training and evaluation data, with perfect ground truth |
| KITTI | Camera + LiDAR detection and tracking |
| nuScenes | Camera + LiDAR + radar, 360° perception |
| View-of-Delft (VoD) | Camera + LiDAR + 4D radar, urban scenes with pedestrians and cyclists |
| TJ4DRadSet | 4D radar + LiDAR + camera, varied lighting |
| K-Radar | 4D radar in fog, rain, snow, day and night |
| BDD100K | Diverse weather and night camera data |
| DENSE / Seeing Through Fog | Real adverse-weather multi-sensor data |
| CODA | Real road corner cases (long tail) |
| IDD (India Driving Dataset) | Indian road conditions and road users |
| DSEC | Real driving event-camera data |

Check each dataset's licence before using it in published results.

## 7. Development Milestones

| Version | Experiments ready |
|---|---|
| V1 (2nd week of January 2027) | E-A in clear weather: camera + LiDAR + radar, tracking, TTC, prediction; radar ablation and E-J on synthetic data |
| V2 | E-B, E-C, E-E, E-J on CARLA / real radar data |
| V3 | E-D, E-F, E-G, E-H, E-I |

## 8. Limitations

- Simulation does not fully represent real sensor physics, calibration errors, road surfaces or human behaviour.
- Simulated driver-interaction data cannot capture the full variety of real medical emergencies; Part 3 outputs are system-confidence states, not diagnoses.
- Evasive-steering and AEB results are valid only within the simulator's vehicle dynamics.
- The prototype is a research and demonstration system, not a production or certified safety system.
