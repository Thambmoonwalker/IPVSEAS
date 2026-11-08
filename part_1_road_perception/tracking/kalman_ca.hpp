// Constant-acceleration Kalman filter in the BEV plane.
// State x = [px, py, vx, vy, ax, ay]^T, measurement z = [px, py]^T.
// Process noise: discretised white-noise jerk with spectral density sigma_jerk^2.
#pragma once

#include <Eigen/Dense>

namespace ipvseas {

using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat6 = Eigen::Matrix<double, 6, 6>;

class KalmanCA {
public:
    KalmanCA() = default;
    KalmanCA(const Eigen::Vector2d& pos, const Eigen::Matrix2d& pos_cov,
             double vel_sigma, double acc_sigma, double sigma_jerk);

    static Mat6 transition(double dt);
    static Mat6 process_noise(double dt, double sigma_jerk);

    void predict(double dt);
    // Innovation and its covariance for a position measurement.
    void innovation(const Eigen::Vector2d& z, const Eigen::Matrix2d& R,
                    Eigen::Vector2d& y, Eigen::Matrix2d& S) const;
    double mahalanobis2(const Eigen::Vector2d& z, const Eigen::Matrix2d& R) const;
    void update(const Eigen::Vector2d& z, const Eigen::Matrix2d& R);
    // EKF update with a radar Doppler measurement:
    //   h(x) = (v - v_s) . u,  u = (p - s) / |p - s|
    // where s, v_s are the radar's position and velocity (same frame as the state).
    // Returns the normalised innovation squared (for consistency checks).
    double update_range_rate(double range_rate, double var, const Eigen::Vector2d& sensor_pos,
                             const Eigen::Vector2d& sensor_vel);
    double predicted_range_rate(const Eigen::Vector2d& sensor_pos, const Eigen::Vector2d& sensor_vel) const;

    // Overwrite the velocity estimate (used for two-point initialisation).
    void set_velocity(const Eigen::Vector2d& v, double vel_sigma);

    const Vec6& state() const { return x_; }
    const Mat6& cov() const { return P_; }
    Eigen::Vector2d pos() const { return x_.head<2>(); }
    Eigen::Vector2d vel() const { return x_.segment<2>(2); }
    Eigen::Vector2d acc() const { return x_.tail<2>(); }
    double sigma_jerk() const { return sigma_jerk_; }

private:
    Vec6 x_ = Vec6::Zero();
    Mat6 P_ = Mat6::Identity();
    double sigma_jerk_ = 3.0;
};

}  // namespace ipvseas
