#include "tracking/kalman_ca.hpp"

namespace ipvseas {

KalmanCA::KalmanCA(const Eigen::Vector2d& pos, const Eigen::Matrix2d& pos_cov,
                   double vel_sigma, double acc_sigma, double sigma_jerk)
    : sigma_jerk_(sigma_jerk) {
    x_.setZero();
    x_.head<2>() = pos;
    P_.setZero();
    P_.topLeftCorner<2, 2>() = pos_cov;
    P_.block<2, 2>(2, 2) = Eigen::Matrix2d::Identity() * vel_sigma * vel_sigma;
    P_.block<2, 2>(4, 4) = Eigen::Matrix2d::Identity() * acc_sigma * acc_sigma;
}

Mat6 KalmanCA::transition(double dt) {
    Mat6 F = Mat6::Identity();
    const double h = 0.5 * dt * dt;
    F(0, 2) = dt; F(0, 4) = h;
    F(1, 3) = dt; F(1, 5) = h;
    F(2, 4) = dt;
    F(3, 5) = dt;
    return F;
}

Mat6 KalmanCA::process_noise(double dt, double sigma_jerk) {
    // Per-axis block for [p, v, a] driven by white jerk.
    const double q = sigma_jerk * sigma_jerk;
    const double d2 = dt * dt, d3 = d2 * dt, d4 = d3 * dt, d5 = d4 * dt;
    Eigen::Matrix3d B;
    B << d5 / 20.0, d4 / 8.0, d3 / 6.0,
         d4 / 8.0,  d3 / 3.0, d2 / 2.0,
         d3 / 6.0,  d2 / 2.0, dt;
    B *= q;
    Mat6 Q = Mat6::Zero();
    const int idx[2][3] = {{0, 2, 4}, {1, 3, 5}};
    for (int axis = 0; axis < 2; ++axis)
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) Q(idx[axis][r], idx[axis][c]) = B(r, c);
    return Q;
}

void KalmanCA::predict(double dt) {
    if (dt <= 0.0) return;
    const Mat6 F = transition(dt);
    x_ = F * x_;
    P_ = F * P_ * F.transpose() + process_noise(dt, sigma_jerk_);
}

void KalmanCA::innovation(const Eigen::Vector2d& z, const Eigen::Matrix2d& R,
                          Eigen::Vector2d& y, Eigen::Matrix2d& S) const {
    y = z - x_.head<2>();
    S = P_.topLeftCorner<2, 2>() + R;
}

double KalmanCA::mahalanobis2(const Eigen::Vector2d& z, const Eigen::Matrix2d& R) const {
    Eigen::Vector2d y;
    Eigen::Matrix2d S;
    innovation(z, R, y, S);
    return y.dot(S.ldlt().solve(y));
}

void KalmanCA::update(const Eigen::Vector2d& z, const Eigen::Matrix2d& R) {
    Eigen::Vector2d y;
    Eigen::Matrix2d S;
    innovation(z, R, y, S);
    // K = P H^T S^-1, with H = [I2 0 0]
    const Eigen::Matrix<double, 6, 2> PHt = P_.leftCols<2>();
    const Eigen::Matrix<double, 6, 2> K = PHt * S.inverse();
    x_ += K * y;
    // Joseph form keeps P symmetric positive-definite.
    Eigen::Matrix<double, 2, 6> H = Eigen::Matrix<double, 2, 6>::Zero();
    H.leftCols<2>().setIdentity();
    const Mat6 IKH = Mat6::Identity() - K * H;
    P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
}

double KalmanCA::predicted_range_rate(const Eigen::Vector2d& sensor_pos,
                                      const Eigen::Vector2d& sensor_vel) const {
    const Eigen::Vector2d d = x_.head<2>() - sensor_pos;
    const double r = d.norm();
    if (r < 1e-3) return 0.0;
    return (x_.segment<2>(2) - sensor_vel).dot(d / r);
}

double KalmanCA::update_range_rate(double range_rate, double var, const Eigen::Vector2d& sensor_pos,
                                   const Eigen::Vector2d& sensor_vel) {
    const Eigen::Vector2d d = x_.head<2>() - sensor_pos;
    const double r = d.norm();
    if (r < 1.0) return 0.0;
    const Eigen::Vector2d u = d / r;
    const Eigen::Vector2d vrel = x_.segment<2>(2) - sensor_vel;
    const double h = vrel.dot(u);
    // Jacobian: dh/dp = (vrel - h u) / r,  dh/dv = u,  dh/da = 0
    Eigen::Matrix<double, 1, 6> H = Eigen::Matrix<double, 1, 6>::Zero();
    H.segment<2>(0) = ((vrel - h * u) / r).transpose();
    H.segment<2>(2) = u.transpose();
    const double y = range_rate - h;
    const double S = (H * P_ * H.transpose())(0, 0) + var;
    const Vec6 K = P_ * H.transpose() / S;
    x_ += K * y;
    const Mat6 IKH = Mat6::Identity() - K * H;
    P_ = IKH * P_ * IKH.transpose() + K * var * K.transpose();
    return y * y / S;
}

void KalmanCA::set_velocity(const Eigen::Vector2d& v, double vel_sigma) {
    x_.segment<2>(2) = v;
    P_.block<2, 4>(2, 0).setZero();
    P_.block<4, 2>(0, 2).setZero();
    P_.block<2, 2>(2, 2) = Eigen::Matrix2d::Identity() * vel_sigma * vel_sigma;
}

}  // namespace ipvseas
