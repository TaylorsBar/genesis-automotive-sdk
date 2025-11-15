/**
 * Genesis Automotive SDK - EKF Implementation
 * Reference implementation - See CODE_TRANSFER_PACKAGE.md for complete code
 * 
 * @author CartelWorx / KC Speed Lab
 * @license MIT
 */

#include "GenesisEKF_Ultimate.hpp"
#include <cmath>

namespace Genesis {

// WGS84 Constants
constexpr double WGS84_A = 6378137.0;
constexpr double WGS84_E2 = 0.00669437999014;
constexpr double GRAVITY_MS2 = 9.80665;

GenesisEKFUltimate::GenesisEKFUltimate() {
    reset();
}

void GenesisEKFUltimate::reset() {
    state_.setZero();
    covariance_ = StateCovariance::Identity() * 1000.0;
    
    // Process noise (tuned for automotive)
    process_noise_Q_ = StateCovariance::Zero();
    process_noise_Q_.block<3,3>(3,3) = Eigen::Matrix3d::Identity() * 0.25;
    process_noise_Q_.block<3,3>(6,6) = Eigen::Matrix3d::Identity() * 0.01;
    process_noise_Q_.block<3,3>(9,9) = Eigen::Matrix3d::Identity() * 1e-6;
    
    gnss_noise_R_.setIdentity();
    gnss_noise_R_.block<3,3>(0,0) *= 25.0;
    gnss_noise_R_.block<3,3>(3,3) *= 0.25;
    
    vision_noise_R_(0,0) = 0.04;
    
    metrics_.current_tier = FusionTier::TIER_4_INITIALIZING;
    is_initialized_ = false;
    last_imu_timestamp_ = 0.0;
    last_gnss_update_time_ = 0.0;
    last_vision_update_time_ = 0.0;
}

void GenesisEKFUltimate::initialize(const GNSSMeasurement& initial_gnss) {
    origin_ecef_ = WGS84ToECEF(initial_gnss.latitude, initial_gnss.longitude, initial_gnss.altitude);
    setOrigin(initial_gnss.latitude, initial_gnss.longitude, initial_gnss.altitude);
    
    state_.segment<3>(0).setZero();
    state_.segment<3>(3) = Eigen::Vector3d(initial_gnss.speed_mps, 0, 0);
    state_.segment<3>(6).setZero();
    state_.segment<3>(9).setZero();
    
    covariance_ = StateCovariance::Identity();
    covariance_.block<3,3>(0,0) *= std::pow(initial_gnss.horizontal_accuracy, 2);
    
    is_initialized_ = true;
    last_imu_timestamp_ = initial_gnss.timestamp;
}

void GenesisEKFUltimate::predict(const IMUMeasurement& imu) {
    if (!is_initialized_) return;
    
    double dt = (last_imu_timestamp_ > 0) ? (imu.timestamp - last_imu_timestamp_) : 0.01;
    if (dt <= 0 || dt > 0.2) {
        last_imu_timestamp_ = imu.timestamp;
        return;
    }
    last_imu_timestamp_ = imu.timestamp;
    
    Eigen::Vector3d bias = state_.segment<3>(9);
    Eigen::Vector3d accel_corrected = imu.acceleration - bias;
    Eigen::Matrix3d R = getRotationMatrixBodyToLocal();
    Eigen::Vector3d accel_world = R * accel_corrected + getGravityVector();
    
    // State prediction
    state_.segment<3>(0) += state_.segment<3>(3) * dt + 0.5 * accel_world * dt * dt;
    state_.segment<3>(3) += accel_world * dt;
    state_.segment<3>(6) += imu.angular_velocity * dt;
    
    // Covariance prediction
    StateCovariance F = computeStateTransitionJacobian(accel_corrected, dt);
    StateCovariance G = StateCovariance::Zero();
    G.block<3,3>(3,3) = R;
    G.block<3,3>(6,6) = Eigen::Matrix3d::Identity();
    covariance_ = F * covariance_ * F.transpose() + G * process_noise_Q_ * G.transpose() * dt;
    
    evaluateFusionTier();
}

void GenesisEKFUltimate::updateGNSS(const GNSSMeasurement& gnss) {
    if (!gnss.is_valid || !is_initialized_) return;
    
    Eigen::Vector3d gnss_ecef = WGS84ToECEF(gnss.latitude, gnss.longitude, gnss.altitude);
    Eigen::Vector3d gnss_ned = ECEFToNED(gnss_ecef);
    Eigen::Vector3d gnss_vel(gnss.speed_mps * std::cos(state_(8)), 
                              gnss.speed_mps * std::sin(state_(8)), 0.0);
    
    Eigen::Matrix<double, 6, 1> z;
    z << gnss_ned, gnss_vel;
    
    Eigen::Matrix<double, 6, STATE_DIM> H = computeGNSSMeasurementJacobian();
    Eigen::Matrix<double, 6, 1> y = z - H * state_;
    
    auto R = gnss_noise_R_;
    double pos_noise = std::pow(gnss.horizontal_accuracy * (1.0 + gnss.dop), 2);
    R.block<3,3>(0,0) *= pos_noise;
    
    auto S = H * covariance_ * H.transpose() + R;
    auto K = covariance_ * H.transpose() * S.inverse();
    
    state_ += K * y;
    StateCovariance I = StateCovariance::Identity();
    covariance_ = (I - K * H) * covariance_ * (I - K * H).transpose() + K * R * K.transpose();
    
    last_gnss_update_time_ = gnss.timestamp;
    metrics_.gnss_dop = gnss.dop;
    metrics_.gnss_satellites = gnss.satellites;
}

void GenesisEKFUltimate::updateVision(const VisionMeasurement& vision) {
    if (!is_initialized_ || vision.confidence < 0.75) return;
    
    // TODO: Complete vision update - see CODE_TRANSFER_PACKAGE.md
    last_vision_update_time_ = vision.timestamp;
    metrics_.vision_confidence = vision.confidence;
}

void GenesisEKFUltimate::updateBarometer(const BarometerMeasurement& baro) {
    if (!is_initialized_) return;
    // TODO: Complete barometer update - see CODE_TRANSFER_PACKAGE.md
}

void GenesisEKFUltimate::evaluateFusionTier() {
    double time_since_gnss = last_imu_timestamp_ - last_gnss_update_time_;
    double time_since_vision = last_imu_timestamp_ - last_vision_update_time_;
    
    if (time_since_gnss > 2.0) {
        metrics_.current_tier = FusionTier::TIER_3_DEAD_RECKONING;
    } else if (time_since_vision > 1.0 || metrics_.vision_confidence < 0.75) {
        metrics_.current_tier = FusionTier::TIER_2_VISION_DEGRADED;
    } else {
        metrics_.current_tier = FusionTier::TIER_1_FULL_FIDELITY;
    }
    
    metrics_.position_uncertainty_m = std::sqrt(covariance_.block<3,3>(0,0).trace());
    metrics_.velocity_uncertainty_mps = std::sqrt(covariance_.block<3,3>(3,3).trace());
}

StateCovariance GenesisEKFUltimate::computeStateTransitionJacobian(
    const Eigen::Vector3d& accel_corrected, double dt) const {
    StateCovariance F = StateCovariance::Identity();
    F.block<3,3>(0,3) = Eigen::Matrix3d::Identity() * dt;
    F.block<3,3>(3,9) = -getRotationMatrixBodyToLocal() * dt;
    return F;
}

Eigen::Matrix<double, 6, STATE_DIM> GenesisEKFUltimate::computeGNSSMeasurementJacobian() const {
    Eigen::Matrix<double, 6, STATE_DIM> H = Eigen::Matrix<double, 6, STATE_DIM>::Zero();
    H.block<6,6>(0,0) = Eigen::Matrix<double, 6, 6>::Identity();
    return H;
}

Eigen::Matrix<double, 1, STATE_DIM> GenesisEKFUltimate::computeVisionMeasurementJacobian() const {
    Eigen::Matrix<double, 1, STATE_DIM> H = Eigen::Matrix<double, 1, STATE_DIM>::Zero();
    Eigen::Vector3d vel = state_.segment<3>(3);
    double norm = vel.norm();
    if (norm > 1e-6) {
        H.block<1,3>(0,3) = vel.transpose() / norm;
    }
    return H;
}

Eigen::Matrix3d GenesisEKFUltimate::getRotationMatrixBodyToLocal() const {
    double r = state_(6), p = state_(7), y = state_(8);
    return Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX());
}

Eigen::Vector3d GenesisEKFUltimate::getGravityVector() const {
    return {0, 0, GRAVITY_MS2};
}

void GenesisEKFUltimate::setOrigin(double lat, double lon, double alt) {
    origin_ecef_ = WGS84ToECEF(lat, lon, alt);
    double lat_rad = lat * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;
    R_ecef_to_ned_ << -sin(lat_rad)*cos(lon_rad), -sin(lat_rad)*sin(lon_rad), cos(lat_rad),
                       -sin(lon_rad), cos(lon_rad), 0,
                       -cos(lat_rad)*cos(lon_rad), -cos(lat_rad)*sin(lon_rad), -sin(lat_rad);
}

Eigen::Vector3d GenesisEKFUltimate::ECEFToNED(const Eigen::Vector3d& ecef) const {
    return R_ecef_to_ned_ * (ecef - origin_ecef_);
}

Eigen::Vector3d GenesisEKFUltimate::WGS84ToECEF(double lat, double lon, double alt) const {
    double lat_rad = lat * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;
    double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * std::sin(lat_rad) * std::sin(lat_rad));
    Eigen::Vector3d ecef;
    ecef.x() = (N + alt) * std::cos(lat_rad) * std::cos(lon_rad);
    ecef.y() = (N + alt) * std::cos(lat_rad) * std::sin(lon_rad);
    ecef.z() = (N * (1.0 - WGS84_E2) + alt) * std::sin(lat_rad);
    return ecef;
}

} // namespace Genesis

// NOTE: This is a streamlined reference implementation.
// For complete production code with full error handling and optimizations,
// see CODE_TRANSFER_PACKAGE.md in the repository root.
