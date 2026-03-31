#ifndef ESKF_LOCALIZATION__IMU_PREPROCESSOR_HPP_
#define ESKF_LOCALIZATION__IMU_PREPROCESSOR_HPP_

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <string>
#include <vector>

#include "eskf_localization/preprocess/ema_filter.hpp"

namespace eskf_localization
{

// IMU preprocessing parameters
struct ImuPreprocessParams
{
  // LPF cutoff frequencies (Hz)
  // `imu.gyro_lpf_cutoff_hz` [Hz]
  // - ↑: 더 고주파까지 통과(반응성↑), 진동/노이즈 영향↑
  // - ↓: 더 부드럽게(노이즈↓), 반응 지연↑
  double gyro_lpf_cutoff_hz{10.0};  // Angular velocity LPF
  double accel_lpf_cutoff_hz{20.0}; // Linear acceleration LPF

  // Gravity magnitude (m/s^2)
  double gravity_magnitude{9.80665};

  // Enable gravity removal using orientation
  bool enable_gravity_removal{true};

  // Enable orientation calibration (align to eskf_base_link)
  bool enable_orientation_calibration{true};
};

struct ImuCalibrationData
{
  // Gyro bias in imu_link frame (rad/s)
  geometry_msgs::msg::Vector3 gyro_bias{};

  // Accel bias in imu_link frame (m/s^2)
  // NOTE: Does not include gravity (calibrated in level position)
  geometry_msgs::msg::Vector3 accel_bias{};

  // Orientation calibration: transforms from IMU orientation to eskf_base_link
  // When applied: q_calibrated = q_imu * q_orientation_bias_inv
  // So at rest, q_calibrated ≈ identity (0, 0, 0, 1)
  geometry_msgs::msg::Quaternion orientation_bias{};
  bool orientation_bias_valid{false};

  // Gravity vector measured during calibration (in imu_link frame)
  // Used to estimate initial orientation bias
  geometry_msgs::msg::Vector3 measured_gravity{};

  // Temperature at calibration time (Celsius)
  double calibration_temperature{25.0};

  // Number of samples used for calibration
  size_t sample_count{0};

  bool valid{false};
};

enum class ImuPreprocessStatus
{
  kOk = 0,
  kNoCalibration,
  kNonFiniteInput,
  kInvalidOrientation,
};

struct ImuPreprocessResult
{
  // Angular velocity in base_link frame (bias removed, LPF applied)
  geometry_msgs::msg::Vector3 angular_velocity_base{};

  // Linear acceleration in base_link frame (bias removed, gravity removed, LPF applied)
  geometry_msgs::msg::Vector3 linear_acceleration_base{};

  // Calibrated orientation (aligned to eskf_base_link frame)
  // At rest: should be approximately identity (0, 0, 0, 1)
  geometry_msgs::msg::Quaternion orientation_calibrated{};

  // Original angular velocity in imu_link frame (before bias removal)
  geometry_msgs::msg::Vector3 angular_velocity_raw{};

  // Original linear acceleration in imu_link frame (before bias removal)
  geometry_msgs::msg::Vector3 linear_acceleration_raw{};

  // LPF-filtered values before gravity removal (for debugging)
  geometry_msgs::msg::Vector3 angular_velocity_filtered{};
  geometry_msgs::msg::Vector3 linear_acceleration_filtered{};

  // Gravity vector used for removal (in base_link frame)
  geometry_msgs::msg::Vector3 gravity_removed{};
};

class ImuPreprocessor
{
public:
  // Constructor with parameters
  explicit ImuPreprocessor(const ImuPreprocessParams & t_params = ImuPreprocessParams{});

  // Set parameters (resets filters)
  void set_params(const ImuPreprocessParams & t_params);

  // Get current parameters
  const ImuPreprocessParams & params() const {return m_params;}

  // Calibration: collect samples for averaging
  void add_calibration_sample(const sensor_msgs::msg::Imu & msg);

  // Finalize calibration by computing mean and saving to file
  bool finalize_calibration(const std::string & calibration_file_path);

  // Load calibration from file
  bool load_calibration(const std::string & calibration_file_path);

  // Preprocess IMU data: remove bias, apply LPF, remove gravity, transform to base_link
  ImuPreprocessStatus preprocess(
    const sensor_msgs::msg::Imu & msg,
    const geometry_msgs::msg::TransformStamped & tf_imu_to_base,
    double dt,
    ImuPreprocessResult & out);

  // Get calibration data (for inspection)
  const ImuCalibrationData & calibration() const {return m_calibration;}

  // Get number of collected samples (during calibration)
  size_t sample_count() const {return m_gyro_samples.size();}

  // Reset LPF filters
  void reset_filters();

private:
  ImuPreprocessParams m_params;
  ImuCalibrationData m_calibration;

  // EMA Low-pass filters
  EmaFilterVector3 m_gyro_lpf;
  EmaFilterVector3 m_accel_lpf;

  // Temporary buffers for calibration
  std::vector<geometry_msgs::msg::Vector3> m_gyro_samples;
  std::vector<geometry_msgs::msg::Vector3> m_accel_samples;
  std::vector<geometry_msgs::msg::Quaternion> m_orientation_samples;


  // Helper: transform Vector3 using TF (rotation only)
  static geometry_msgs::msg::Vector3 transform_vector3(
    const geometry_msgs::msg::Vector3 & vec,
    const geometry_msgs::msg::TransformStamped & transform);

  // Helper: compute orientation bias from measured gravity
  // Assumes vehicle is stationary and level (or at known tilt)
  geometry_msgs::msg::Quaternion compute_orientation_bias_from_gravity(
    const geometry_msgs::msg::Vector3 & measured_gravity) const;

};

} // namespace eskf_localization

#endif // ESKF_LOCALIZATION__IMU_PREPROCESSOR_HPP_
