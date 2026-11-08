# IPVSEAS: System Architecture

## 1. Design Principles

1. **Human in control by default.** The system assists. It does not drive.
2. **Predict, don't just detect.** Every tracked object carries a predicted future, not only a current position.
3. **Never report "clear" from a blind sensor.** Each sensor reports a confidence, and low confidence raises caution.
4. **Privacy by design.** All cameras face outward. Driver state comes only from interaction signals.
5. **Graded, explainable response.** Every intervention is a defined escalation level with a logged reason.
6. **Modular and testable.** Each module runs and is evaluated on its own before integration.
7. **Fail-safe.** A failing module degrades the system's confidence; it must never produce an unsafe "all clear".

## 2. Top-Level Architecture

```text
 ┌──────────────────────────── SENSOR SUITE ────────────────────────────┐
 │ Outward: front/rear/side cameras · LiDAR · 4D imaging radar (front,  │
 │          rear, corners) · event camera                               │
 │ Vehicle: speed, yaw rate, steering angle, pedals, gear, horn         │
 │ Cabin (non-visual): steering grip/touch, seat pressure, ack. button  │
 └──────────────────────────────────────────────────────────────────────┘
          │                         │                          │
          ▼                         ▼                          ▼
 ┌─────────────────┐     ┌────────────────────┐     ┌─────────────────────┐
 │ PART 1          │     │ PART 2             │     │ PART 3              │
 │ Road perception │◄───►│ Weather &          │     │ Driver interaction  │
 │ & collision     │conf.│ blind-spot safety  │     │ & emergency         │
 │ prediction      │     │                    │     │ assistance          │
 └────────┬────────┘     └─────────┬──────────┘     └──────────┬──────────┘
          │ ObjectList, Hazards    │ SensorConfidence,         │ DriverState
          │ (TTC, risk, traj.)     │ LaneOccupancy, RearRisk   │
          └────────────────────────┼───────────────────────────┘
                                   ▼
                  ┌──────────────────────────────────┐
                  │      SAFETY DECISION ENGINE      │
                  │ risk fusion · escalation L0–L6   │
                  │ brake-vs-steer · override policy │
                  └───────────────┬──────────────────┘
                                  ▼
       HMI (visual / audible / haptic) · AEB · evasive steering ·
       rear automatic braking · MRM · emergency notification (simulated)
                                  │
                         Event logger (no personal data)

 ┌──────────────────────── END-TO-END ARM (research) ───────────────────┐
 │ Same sensor inputs → learned risk / decision → compared with modular │
 │ pipeline; rule-based supervisor can veto                             │
 └──────────────────────────────────────────────────────────────────────┘
```

## 3. Shared Data Interfaces

Defining these early lets the three parts be built independently and then plugged together.

| Message | Producer | Key fields |
|---|---|---|
| `TrackedObject` | Part 1 | id, class (or `unknown_obstacle`), BEV state `[x, y, vx, vy, ax, ay]`, covariance, detection confidence, OOD score |
| `PredictedTrajectory` | Part 1 | object id, future poses over horizon (e.g. 3 s), probability |
| `Hazard` | Part 1 | object id, TTC, path-conflict point, collision probability |
| `SensorConfidence` | Part 2 | per-sensor reliability (0–1), estimated visibility range, weather class |
| `LaneOccupancy` | Part 2 | left / right / rear zones: free, occupied or uncertain, plus closing speed |
| `RearRisk` | Part 2 | rear-object distance, closing speed, rear TTC |
| `DriverState` | Part 3 | `normal`, `unusual`, `possibly_unresponsive` or `suspected_emergency`, confidence, last acknowledgement time |
| `SafetyAction` | Decision engine | level L0–L6, action, reason, timestamp |

## 4. Part 1: Road Perception and Collision Prediction

### 4.1 Sensor pipelines
- **Camera:** YOLO 2D detection and classification (vehicles, pedestrians, cyclists, animals); lane and road-marking context.
- **LiDAR:** ground removal, clustering, then **class-agnostic obstacles**; a 3D detector (OpenPCDet candidate) for known classes.
- **4D imaging radar:** range, azimuth, elevation and radial velocity per point; strong for closing speed and for all-weather detection (see 4.6).
- **Event camera:** event-stream motion detection for fast-emerging objects, with lower latency than frame-based detection.

### 4.2 Fusion and tracking (C++)
- Spatial calibration and transforms into a common vehicle (BEV) frame; timestamp alignment.
- Association (gating plus Hungarian / nearest-neighbour), then Kalman filter / EKF tracking with a constant-acceleration or CTRV motion model.
- Fusion weights come from Part 2's `SensorConfidence`.

### 4.3 Behaviour and trajectory prediction
- Physics-based baseline: constant velocity / acceleration rollout.
- Intent cues: deceleration (sudden braking), lateral velocity toward our lane (cut-in), pedestrian heading toward the kerb (crossing).
- Learned predictor (LSTM / Transformer) as an upgrade, trained in Python and exported to ONNX for C++.

### 4.4 Collision-risk assessment
- **TTC** = d / v_closing (for v_closing > 0); an enhanced TTC also includes relative acceleration.
- Path-conflict check between our predicted path and each object's predicted trajectory.
- Collision probability from state covariance (uncertainty-aware).

### 4.5 Long-tail handling
- Any LiDAR/radar obstacle in our path is a hazard **even if the camera has no class for it**.
- OOD scoring on camera detections: low-confidence or unfamiliar detections raise caution instead of being discarded.

### 4.6 4D imaging radar pipeline (C++) (implemented in V1: `radar/radar_processor.*`; details in `part1_specification.md` 4.4–4.6)
Each detection: range r, azimuth θ, elevation φ, radial velocity v_r, and reflection strength (RCS / SNR).

1. **Polar → Cartesian** in the vehicle frame: x = r·cosφ·cosθ, y = r·cosφ·sinθ, z = r·sinφ.
2. **Ego-motion compensation of Doppler.** For a forward-facing radar on a vehicle moving at v_ego, a static point returns v_r ≈ −v_ego·cosθ·cosφ. The compensated velocity v_comp = v_r + v_ego·cosθ·cosφ is ≈ 0 for static points, which splits the cloud into static and moving points.
3. **Multi-frame accumulation** (ego-motion corrected) over a few sweeps to densify the sparse cloud.
4. **Ghost / multipath filtering:** reject points below the road surface, mirror-image points behind guardrails, and points with implausible RCS for their range.
5. **Elevation gating for stationary objects:** static clusters whose height lies in the drivable envelope (road level up to vehicle height) are kept as obstacles; clusters that sit only above clearance height (bridges, gantries) or flat on the road (manhole covers, plates) are rejected.
6. **Clustering / detection:** DBSCAN clustering as a baseline; a learned radar point-cloud detector (pillar-based) as an upgrade.
7. **Tracking input:** clusters enter the KF/EKF with both position and measured radial velocity as measurements. Tangential velocity still comes from tracking over time.
8. **Micro-Doppler cue:** velocity spread within a cluster helps flag pedestrians and animals.

## 5. Part 2: Weather and Blind-Spot Safety

### 5.1 Weather perception and sensor confidence
- Visibility estimation from camera contrast / dark-channel statistics and LiDAR return-density drop-off.
- Per-sensor confidence model: for example, camera confidence falls in fog and at night, LiDAR confidence falls in heavy rain and fog, and 4D radar stays the most stable (but its ghost rate rises in tunnels and near guardrails).
- In dense fog the fusion shifts weight toward 4D radar for detection and height, keeping the camera for classification whenever it is still confident.
- **Physics-based degradation models** applied to simulated LiDAR and radar (CARLA's built-in weather mainly affects the camera image).

### 5.2 Surrounding traffic (TCAS-inspired)
- 360° BEV occupancy around the ego vehicle from all sensors.
- Zones: front, rear, left/right adjacent lanes and the blind spots.
- Conflict prediction for cut-ins, rear approach and side/intersection approach.

### 5.3 Rear automatic braking
- **Reverse mode:** at low speed in reverse, rear/corner 4D radars + rear camera + LiDAR detect obstacles and pedestrians; elevation helps reject kerbs and ground clutter. Brake if rear TTC < threshold.
- **Rear-aware forward braking:** provides `RearRisk` so the decision engine knows whether hard forward braking risks a rear-end collision, and whether steering is preferable.

## 6. Part 3: Driver Incapacitation and Emergency Assistance

### 6.1 Signals (no face camera)
Steering angle and rate · steering grip/touch · accelerator and brake position · horn state and duration · seat-pressure distribution · acknowledgement button · response to haptic prompts.

### 6.2 Context-aware anomaly detection
The same signal means different things in different contexts:
- No steering corrections on a straight road: normal. No steering in a curve or during lane drift: suspicious.
- Long horn activation together with no pedal input and a shifted seat-pressure pattern: strong indicator.
- The model uses road context (curvature, traffic, speed) from Parts 1 and 2.

### 6.3 State machine

```text
NORMAL ──anomaly──► UNUSUAL ──persists──► POSSIBLY_UNRESPONSIVE
   ▲                   │                          │
   └──── driver ack ◄──┴─── haptic + audible ─────┤
                                                  │ no ack within T
                                                  ▼
                                        SUSPECTED_EMERGENCY
                                                  │
                              MRM (simulated) + emergency notification
```

### 6.4 Minimum-risk manoeuvre
Hazard lights, gradual deceleration, lane keeping, and if Part 2 confirms it is safe, a move to the shoulder; stop and send the notification. The behaviour differs by context: if the vehicle is already stopped, only the notification is sent.

## 7. Safety Decision Engine

### 7.1 Escalation ladder

| Level | Action |
|---|---|
| L0 | Monitor |
| L1 | Information cue (icon, soft tone) |
| L2 | Warning: visual, audible, haptic (steering wheel / seat patterns) |
| L3 | Brake assist / pre-fill |
| L4 | Automatic emergency braking |
| L5 | Emergency evasive steering |
| L6 | Minimum-risk manoeuvre + emergency notification |

### 7.2 Brake vs steer decision
With a simplified point-mass model, at relative speed v:

- distance needed to avoid the obstacle by braking: d_brake = v² / (2·a_x)
- distance needed to avoid it by steering (lateral offset w): d_steer = v · √(2w / a_y)

Steering needs less distance than braking when v > 2·a_x·√(2w / a_y).
*Example:* a_x = 8 m/s², a_y = 6 m/s², w = 1.8 m gives v ≈ 12.4 m/s (≈ 45 km/h relative speed).

Above this crossover, if braking alone can no longer avoid the collision, the engine considers **L5 evasive steering**, but only if `LaneOccupancy` reports the escape zone as **free** (not just "not detected"). Otherwise it stays at maximum braking. This model ignores reaction and actuator delays and tyre limits; a refined version is part of the research.

### 7.3 Haptic patterns (to be validated)
Short single pulse for the blind spot · repeated pulses for rising collision risk · continuous strong vibration for imminent collision · rhythmic prompt for the responsiveness check.

## 8. End-to-End Arm

- Input: the same camera / LiDAR (and optionally event) streams.
- Output: a collision-risk score (first) and possibly an action proposal (later).
- Baselines to study: CARLA end-to-end models such as TransFuser and InterFuser.
- Always runs **behind a rule-based supervisor**; used for comparison, not as the safety core.
- Hardware note: an 8 GB laptop GPU is enough for inference and fine-tuning small models, not for training large end-to-end models from scratch.

## 9. Language Split

| C++ (~60%) | Python (~40%) |
|---|---|
| Tracking (KF/EKF), association, fusion | YOLO / 3D-detector training and evaluation |
| 4D radar pipeline (Doppler compensation, ghost filtering, clustering) | Radar degradation / multipath models, radar detector training |
| Trajectory rollout, TTC, risk | CARLA scenario scripting and data collection |
| Safety decision engine, brake-vs-steer | Weather degradation models, datasets |
| Driver-state machine | Anomaly-detection model training |
| ONNX Runtime inference | End-to-end arm, plots, reports |

## 10. Development Environment

Windows (MSVC, CMake, vcpkg) for the C++ core and Python tooling; WSL2/Ubuntu for OpenPCDet and other Linux-first deep-learning libraries; CARLA on the RTX 5050 laptop.
