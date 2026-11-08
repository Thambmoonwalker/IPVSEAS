# Sensor Fusion

Late camera–LiDAR fusion: project LiDAR clusters into the image, IoU + Hungarian association with YOLO boxes; LiDAR-only objects stay `unknown_obstacle`; camera-only objects get a monocular ground-plane range.

Code: `camera_lidar_fusion.hpp/.cpp`. Status: **implemented (V1)**.
