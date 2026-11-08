# Radar

Front 4D imaging radar (range, azimuth, elevation, Doppler) processing:
polar → Cartesian, ego-motion Doppler compensation, multipath-ghost filter, range-scaled
DBSCAN with Doppler agreement, elevation gating of stationary objects (truck kept, gantry /
manhole cover rejected), near-field and FOV-edge rules.

Code: `radar_processor.hpp/.cpp`; fusion in `../sensor_fusion/radar_fusion.*`; Doppler EKF
update in `../tracking/kalman_ca.*`. Details: `docs/part1_specification.md`, sections 4.4–4.6.

Status: **implemented (V1)**, tested on synthetic data. Adverse-weather radar models and
corner radars: Part 2.
