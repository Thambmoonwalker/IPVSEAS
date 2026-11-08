# IPVSEAS: Problem Statement

## 1. Background

Most serious road accidents involve hazards that a human driver either **sees too late** or **cannot see at all**:

- a stopped or broken-down vehicle ahead, often without hazard lights;
- a vehicle that brakes suddenly or cuts into our lane;
- a pedestrian or animal entering the road, especially at night;
- fog, smog, rain, dust or glare that shrinks the driver's visibility;
- a vehicle hidden in a blind spot or approaching fast from behind;
- rare, unexpected hazards: fallen loads, overturned vehicles, wrong-way drivers, unlit slow vehicles;
- a driver who suddenly becomes unable to drive because of a medical emergency or loss of consciousness.

Existing driver-assistance systems mostly **react** to an obstacle once it is detected, often from a single sensor, and driver-monitoring systems usually rely on driver-facing cameras that record the driver's face.

Fully autonomous driving removes the human from the loop. That is a different goal from this project: IPVSEAS keeps the human driving experience and protects the driver from what they cannot see, predict or respond to.

## 2. Problem Definition

Design and evaluate, in simulation, an intelligent vehicle safety layer that:

1. **Predicts** collisions with vehicles, pedestrians, animals and obstacles, including rare long-tail hazards, before they become unavoidable.
2. **Maintains perception** in fog, smog, rain, night and glare, and in every direction around the vehicle (blind spots, rear, adjacent lanes), while honestly reporting its own confidence.
3. **Detects driver incapacitation** without any driver-facing camera, using only driver–vehicle interaction signals (steering, pedals, horn, seat pressure) and responses to haptic and audible prompts.
4. **Responds proportionately** through a graded escalation: warnings, brake assist, automatic emergency braking, emergency evasive steering, rear automatic braking, and a minimum-risk manoeuvre with emergency notification.

## 3. Problem Breakdown

| Part | Problems covered |
|---|---|
| **Part 1: Road-user and obstacle collision prediction** | Stopped / slow / suddenly braking vehicles · lane-change prediction · pedestrian and animal path prediction · long-tail obstacles · fast-emerging hazards (event camera) |
| **Part 2: Adverse weather and blind-spot safety** | Fog, smog, rain, night, glare · 4D imaging radar as the all-weather backbone · per-sensor confidence · 360° traffic picture (TCAS-inspired) · blind spot and cut-in warnings · rear automatic braking |
| **Part 3: Driver incapacitation and emergency assistance** | Privacy-preserving abnormal-interaction detection (e.g. horn held by an unconscious driver) · responsiveness check · minimum-risk manoeuvre · emergency notification |
| **Safety Decision Engine** | Combines all parts; selects warning vs AEB vs evasive steering vs MRM |

## 4. Central Research Question

> How can predictive environmental perception, weather-aware multi-sensor confidence and privacy-preserving driver-responsiveness monitoring be combined into one context-aware safety system that intervenes earlier and more appropriately than detection-only systems, while leaving the driver in control during normal driving?

### Supporting questions

1. **Prediction:** Does trajectory-based risk assessment give longer warning lead times than detection-and-distance thresholds, without more false alarms?
2. **Weather:** Can per-sensor confidence estimation prevent "false clear road" failures in fog, rain and night?
3. **Long tail:** How much does class-agnostic (geometry-based) obstacle detection reduce misses on rare hazards that the camera detector cannot classify?
4. **Driver monitoring:** Can incapacitation be detected reliably from interaction signals alone, with no face camera?
5. **Intervention:** When should the system brake, and when should it steer? How does knowledge of the rear and adjacent lanes change that choice?
6. **End-to-end vs modular:** Where does an end-to-end network outperform the modular pipeline, and where does it fail?
7. **Event cameras:** Do event cameras shorten detection latency for sudden hazards and improve robustness to glare?
8. **4D imaging radar:** Does radar elevation solve the stationary-vehicle problem, and can camera + 4D radar match camera + LiDAR, especially in fog and rain?

## 5. Scope

- Simulation-based (CARLA), complemented by public datasets for validation.
- No connection to, or control of, any real vehicle's steering, braking or emergency-calling systems.
- No driver-facing camera and no storage of facial or personally identifying driver data.
- Driver-monitoring outputs are **system-confidence states**, not medical diagnoses.

## 6. Expected Outcome

An integrated research prototype that demonstrates, with measured results:

- predictive collision warnings for vehicles, pedestrians and animals;
- weather-aware, confidence-reporting perception and 360° traffic awareness;
- privacy-preserving driver incapacitation detection;
- graded interventions (AEB, evasive steering, rear automatic braking, MRM) in simulation;
- a comparison of modular vs end-to-end approaches, and of frame vs event cameras;
- a reproducible GitHub repository, demo videos and a technical report.
