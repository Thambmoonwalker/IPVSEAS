// IPVSEAS Part 1 - shared data types.
//
// Frames (ISO 8855, right-handed):
//   vehicle frame : origin at the ego vehicle centre on the ground (CARLA actor origin),
//                   x forward, y left, z up
//   world frame   : fixed, x/y on the ground plane, z up, yaw counter-clockwise from +x
// Tracking runs in the WORLD frame so that a stationary object has zero velocity
// regardless of how the ego vehicle moves.
// NOTE: CARLA uses a LEFT-handed frame (x forward, y RIGHT, z up). Convert with y -> -y
// and yaw -> -yaw when reading CARLA data (done in the Python recorder).
#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace ipvseas {

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kPi = 3.14159265358979323846;

enum class ObjectClass { Unknown, Car, Truck, Bus, Motorcycle, Bicycle, Pedestrian, Animal };

enum class ClassGroup { Unknown, Vehicle, VulnerableRoadUser, Animal };

inline ClassGroup group_of(ObjectClass c) {
    switch (c) {
        case ObjectClass::Car:
        case ObjectClass::Truck:
        case ObjectClass::Bus:
        case ObjectClass::Motorcycle: return ClassGroup::Vehicle;
        case ObjectClass::Bicycle:
        case ObjectClass::Pedestrian: return ClassGroup::VulnerableRoadUser;
        case ObjectClass::Animal: return ClassGroup::Animal;
        default: return ClassGroup::Unknown;
    }
}

inline ObjectClass class_from_string(const std::string& s) {
    if (s == "car") return ObjectClass::Car;
    if (s == "truck") return ObjectClass::Truck;
    if (s == "bus") return ObjectClass::Bus;
    if (s == "motorcycle") return ObjectClass::Motorcycle;
    if (s == "bicycle") return ObjectClass::Bicycle;
    if (s == "pedestrian") return ObjectClass::Pedestrian;
    if (s == "animal") return ObjectClass::Animal;
    return ObjectClass::Unknown;
}

inline const char* to_string(ObjectClass c) {
    switch (c) {
        case ObjectClass::Car: return "car";
        case ObjectClass::Truck: return "truck";
        case ObjectClass::Bus: return "bus";
        case ObjectClass::Motorcycle: return "motorcycle";
        case ObjectClass::Bicycle: return "bicycle";
        case ObjectClass::Pedestrian: return "pedestrian";
        case ObjectClass::Animal: return "animal";
        default: return "unknown_obstacle";
    }
}

// Default footprint (length, width) in metres when a sensor gives no size.
inline Eigen::Vector2d default_footprint(ObjectClass c) {
    switch (c) {
        case ObjectClass::Car: return {4.5, 1.9};
        case ObjectClass::Truck: return {9.0, 2.5};
        case ObjectClass::Bus: return {11.0, 2.5};
        case ObjectClass::Motorcycle: return {2.1, 0.8};
        case ObjectClass::Bicycle: return {1.8, 0.6};
        case ObjectClass::Pedestrian: return {0.6, 0.6};
        case ObjectClass::Animal: return {1.8, 0.7};
        default: return {1.0, 1.0};
    }
}

// Typical height (m) when a sensor gives none (used by the synthetic radar model).
inline double default_height(ObjectClass c) {
    switch (c) {
        case ObjectClass::Truck: return 3.5;
        case ObjectClass::Bus: return 3.2;
        case ObjectClass::Pedestrian: return 1.7;
        case ObjectClass::Bicycle: case ObjectClass::Motorcycle: return 1.5;
        case ObjectClass::Animal: return 1.4;
        default: return 1.5;
    }
}

enum class Sensor { Camera, Lidar, Radar, Fused, Synthetic };

// One object measurement, already expressed in the WORLD frame.
struct Detection {
    double t = 0.0;                                         // timestamp [s]
    Eigen::Vector2d pos = Eigen::Vector2d::Zero();          // BEV centre [m]
    Eigen::Matrix2d cov = Eigen::Matrix2d::Identity() * 0.04;  // position covariance [m^2]
    ObjectClass cls = ObjectClass::Unknown;
    double confidence = 1.0;
    double length = kNaN, width = kNaN;                     // footprint if measured
    double yaw = kNaN;                                      // heading if measured [rad]
    Sensor source = Sensor::Synthetic;

    // Radar Doppler measurement (optional): radial velocity of the object relative to the
    // sensor, measured along the line of sight (positive = moving away).
    bool has_range_rate = false;
    double range_rate = 0.0;                                // [m/s]
    double range_rate_var = 0.04;                           // [m^2/s^2]
    Eigen::Vector2d sensor_pos = Eigen::Vector2d::Zero();   // radar position (same frame as pos)
    Eigen::Vector2d sensor_vel = Eigen::Vector2d::Zero();   // radar velocity (same frame)

    // False for a radar-only STATIONARY return whose height could not be confirmed to lie
    // between road level and vehicle height (could be a manhole cover or a bridge).
    bool height_confirmed = true;
};

// Ego vehicle state in the WORLD frame (reference point = vehicle centre).
struct EgoState {
    double t = 0.0;
    double x = 0.0, y = 0.0, yaw = 0.0;   // pose
    double speed = 0.0;                   // [m/s]
    double yaw_rate = 0.0;                // [rad/s]
    double accel = 0.0;                   // longitudinal [m/s^2]
    double length = 4.7, width = 1.9;
};

enum class WarningLevel { None = 0, Caution = 1, Warning = 2, Critical = 3 };

inline const char* to_string(WarningLevel w) {
    switch (w) {
        case WarningLevel::Caution: return "CAUTION";
        case WarningLevel::Warning: return "WARNING";
        case WarningLevel::Critical: return "CRITICAL";
        default: return "NONE";
    }
}

inline double wrap_angle(double a) {
    while (a > kPi) a -= 2.0 * kPi;
    while (a < -kPi) a += 2.0 * kPi;
    return a;
}

}  // namespace ipvseas
