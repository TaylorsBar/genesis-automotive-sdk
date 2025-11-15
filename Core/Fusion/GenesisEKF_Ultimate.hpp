/**
 * Genesis Automotive SDK - Ultimate Extended Kalman Filter
 * 
 * Production-grade 12-state EKF for automotive telemetry with sensor fusion
 * Matches Genesis Droplet live implementation
 * 
 * @author CartelWorx / KC Speed Lab
 * @license MIT
 * @version 1.0.0-alpha
 */

#ifndef GENESIS_EKF_ULTIMATE_HPP
#define GENESIS_EKF_ULTIMATE_HPP

#include <Eigen/Dense>
#include <chrono>
#include <memory>

namespace Genesis {

// State vector dimensions: [px, py, pz, vx, vy, vz, roll, pitch, yaw, bias_ax, bias_ay, bias_az]
constexpr size_t STATE_DIM = 12;
constexpr size_t GNSS_MEAS_DIM = 6;  // [px, py, pz, vx, vy, vz]
constexpr size_t VISION_MEAS_DIM = 1; // [velocity_magnitude]

using StateVector = Eigen::Matrix<double, STATE_DIM, 1>;
using StateCovariance = Eigen::Matrix<double, STATE_DIM, STATE_DIM>;

/**
 * @brief IMU measurement structure
 * Matches 100Hz sampling rate from Genesis Droplet
 */
struct IMUMeasurement {
    double timestamp;                    // Unix timestamp
    Eigen::Vector3d acceleration;        // m/s^2 in body frame
    Eigen::Vector3d angular_velocity;    // rad/s [roll_rate, pitch_rate, yaw_rate]
};

/**
 * @brief GNSS measurement structure  
 * Supports 10-25Hz update rate with accuracy metadata
 */
struct GNSSMeasurement {
    double timestamp;
    double latitude;                     // WGS84 degrees
    double longitude;                    // WGS84 degrees
    double altitude;                     // meters above sea level
    double speed_mps;                    // ground speed m/s
    double horizontal_accuracy;          // meters (±)
    double dop;                          // Dilution of Precision
    int satellites;                      // Number of satellites in view
    bool is_valid;                       // Quality flag
};

/**
 * @brief Vision-based velocity measurement
 * From optical flow / computer vision at 30Hz
 */
struct VisionMeasurement {
    double timestamp;
    double velocity_mps;                 // Ground-truth velocity from optical flow
    double confidence;                   // 0.0-1.0 confidence score
};

/**
 * @brief Barometric altitude measurement
 */
struct BarometerMeasurement {
    double timestamp;
    double altitude_m;                   // Barometric altitude
    double accuracy_m;                   // Measurement uncertainty
};

/**
 * @brief Fusion quality tiers
 * Implements tiered fallback system for degraded sensor conditions
 */
enum class FusionTier {
    TIER_1_FULL_FIDELITY,       // GNSS + IMU + Vision (±0.01s, <1m accuracy)
    TIER_2_VISION_DEGRADED,     // GNSS + IMU only (±0.05s, <5m accuracy)
    TIER_3_DEAD_RECKONING,      // IMU only (±0.2s, exponential drift)
    TIER_4_INITIALIZING         // Waiting for first GNSS lock
};

/**
 * @brief Fusion metrics for monitoring and diagnostics
 * Exposed to Genesis Droplet dashboard
 */
struct FusionMetrics {
    FusionTier current_tier;
    double position_uncertainty_m;       // Derived from covariance matrix
    double velocity_uncertainty_mps;     // Velocity confidence
    double gnss_dop;                     // Current GNSS quality
    double vision_confidence;            // Vision system confidence
    int gnss_satellites;                 // Satellite count
    std::chrono::milliseconds time_in_tier_3;  // Dead reckoning duration
};

/**
 * @class GenesisEKFUltimate
 * @brief Production-grade Extended Kalman Filter for automotive telemetry
 * 
 * Implements:
 * - 12-state EKF with IMU bias estimation
 * - WGS84 ↔ ECEF ↔ NED coordinate transforms
 * - Adaptive noise matrices based on sensor quality
 * - Tiered fallback system for robust operation
 * - High-frequency prediction (100+ Hz) with asynchronous sensor updates
 */
class GenesisEKFUltimate {
public:
    /**
     * @brief Constructor - initializes EKF with default parameters
     */
    GenesisEKFUltimate();
    
    /**
     * @brief Reset filter to initial state
     */
    void reset();
    
    /**
     * @brief Initialize EKF with first GNSS reading
     * Establishes local NED coordinate frame origin
     */
    void initialize(const GNSSMeasurement& initial_gnss);
    
    /**
     * @brief EKF prediction step using IMU data
     * Called at high frequency (100+ Hz)
     * @param imu IMU measurement
     */
    void predict(const IMUMeasurement& imu);
    
    /**
     * @brief EKF update step using GNSS measurement
     * Called at 10-25 Hz
     * @param gnss GNSS measurement
     */
    void updateGNSS(const GNSSMeasurement& gnss);
    
    /**
     * @brief EKF update step using vision measurement
     * Called at ~30 Hz when vision is available
     * @param vision Vision measurement
     */
    void updateVision(const VisionMeasurement& vision);
    
    /**
     * @brief EKF update step using barometer
     * @param baro Barometer measurement
     */
    void updateBarometer(const BarometerMeasurement& baro);
    
    /**
     * @brief Get current state estimate
     * @return 12-dimensional state vector
     */
    StateVector getState() const { return state_; }
    
    /**
     * @brief Get current fusion metrics
     * @return Fusion quality metrics
     */
    FusionMetrics getMetrics() const { return metrics_; }
    
    /**
     * @brief Get current fusion tier
     * @return Active fusion tier
     */
    FusionTier getCurrentTier() const { return metrics_.current_tier; }
    
    /**
     * @brief Check if filter is initialized
     * @return true if initialized with GNSS origin
     */
    bool isInitialized() const { return is_initialized_; }

private:
    // State estimation
    StateVector state_;                  // EKF state vector
    StateCovariance covariance_;         // State covariance matrix P
    
    // Noise matrices (adaptive)
    StateCovariance process_noise_Q_;    // Process noise
    Eigen::Matrix<double, GNSS_MEAS_DIM, GNSS_MEAS_DIM> gnss_noise_R_;
    Eigen::Matrix<double, VISION_MEAS_DIM, VISION_MEAS_DIM> vision_noise_R_;
    
    // Coordinate system
    Eigen::Vector3d origin_ecef_;        // Local frame origin in ECEF
    Eigen::Matrix3d R_ecef_to_ned_;      // Rotation matrix ECEF → NED
    
    // Timing and state
    double last_imu_timestamp_;
    double last_gnss_update_time_;
    double last_vision_update_time_;
    bool is_initialized_;
    
    // Tier management
    FusionMetrics metrics_;
    std::chrono::steady_clock::time_point tier3_start_time_;
    
    // EKF mathematics - private helpers
    StateCovariance computeStateTransitionJacobian(const Eigen::Vector3d& accel_corrected, double dt) const;
    Eigen::Matrix<double, GNSS_MEAS_DIM, STATE_DIM> computeGNSSMeasurementJacobian() const;
    Eigen::Matrix<double, VISION_MEAS_DIM, STATE_DIM> computeVisionMeasurementJacobian() const;
    
    // Tier evaluation
    void evaluateFusionTier();
    
    // Coordinate transformations
    Eigen::Vector3d WGS84ToECEF(double lat, double lon, double alt) const;
    Eigen::Vector3d ECEFToNED(const Eigen::Vector3d& ecef) const;
    void setOrigin(double lat, double lon, double alt);
    
    // Helper functions
    Eigen::Matrix3d getRotationMatrixBodyToLocal() const;
    Eigen::Vector3d getGravityVector() const;
};

} // namespace Genesis

#endif // GENESIS_EKF_ULTIMATE_HPP
