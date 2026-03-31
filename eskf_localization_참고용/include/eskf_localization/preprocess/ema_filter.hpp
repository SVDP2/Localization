#ifndef ESKF_LOCALIZATION__EMA_FILTER_HPP_
#define ESKF_LOCALIZATION__EMA_FILTER_HPP_

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <cmath>

namespace eskf_localization
{

// Exponential Moving Average (EMA) Low-Pass Filter for Vector3
//
// EMA formula: y[n] = α * x[n] + (1 - α) * y[n-1]
// where α = dt / (τ + dt), τ = time constant (cutoff_freq = 1 / (2π * τ))
//
// Advantages over Butterworth:
// - Lower latency (group delay ≈ τ)
// - Simpler implementation
// - No ringing/overshoot
class EmaFilterVector3
{
public:
  EmaFilterVector3() = default;

  // Initialize with cutoff frequency (Hz)
  // tau = 1 / (2π * cutoff_freq)
  explicit EmaFilterVector3(double t_cutoff_freq_hz)
  : m_tau(1.0 / (2.0 * M_PI * t_cutoff_freq_hz)) {}

  // Set cutoff frequency (Hz)
  void set_cutoff_freq(double t_cutoff_freq_hz)
  {
    m_tau = 1.0 / (2.0 * M_PI * t_cutoff_freq_hz);
  }

  // Get time constant (seconds)
  double tau() const {return m_tau;}

  // Reset filter state
  void reset() {m_initialized = false;}

  // Check if filter is initialized
  bool initialized() const {return m_initialized;}

  // Get current filtered value
  const geometry_msgs::msg::Vector3 & value() const {return m_value;}

  // Apply filter with variable dt
  // Returns filtered value
  geometry_msgs::msg::Vector3 update(
    const geometry_msgs::msg::Vector3 & t_input,
    double t_dt)
  {
    if (!m_initialized) {
      m_value = t_input;
      m_initialized = true;
      return m_value;
    }

    // Clamp dt to avoid numerical issues
    const double dt = std::clamp(t_dt, 1e-6, 1.0);

    // Compute alpha: α = dt / (τ + dt)
    // As dt → 0, α → 0 (more smoothing)
    // As dt → ∞, α → 1 (less smoothing)
    const double alpha = dt / (m_tau + dt);

    // EMA: y = α * x + (1 - α) * y_prev
    m_value.x = alpha * t_input.x + (1.0 - alpha) * m_value.x;
    m_value.y = alpha * t_input.y + (1.0 - alpha) * m_value.y;
    m_value.z = alpha * t_input.z + (1.0 - alpha) * m_value.z;

    return m_value;
  }

private:
  double m_tau{0.01};                    // Time constant (seconds), default ~15.9 Hz
  bool m_initialized{false};
  geometry_msgs::msg::Vector3 m_value{}; // Current filtered value
};

// EMA filter for scalar values
class EmaFilterScalar
{
public:
  EmaFilterScalar() = default;

  explicit EmaFilterScalar(double t_cutoff_freq_hz)
  : m_tau(1.0 / (2.0 * M_PI * t_cutoff_freq_hz)) {}

  void set_cutoff_freq(double t_cutoff_freq_hz)
  {
    m_tau = 1.0 / (2.0 * M_PI * t_cutoff_freq_hz);
  }

  double tau() const {return m_tau;}

  void reset() {m_initialized = false;}

  bool initialized() const {return m_initialized;}

  double value() const {return m_value;}

  double update(double t_input, double t_dt)
  {
    if (!m_initialized) {
      m_value = t_input;
      m_initialized = true;
      return m_value;
    }

    const double dt = std::clamp(t_dt, 1e-6, 1.0);
    const double alpha = dt / (m_tau + dt);
    m_value = alpha * t_input + (1.0 - alpha) * m_value;

    return m_value;
  }

private:
  double m_tau{0.01};
  bool m_initialized{false};
  double m_value{0.0};
};

// Quaternion utilities for orientation processing
namespace quat_utils
{

// Check if quaternion is valid (non-zero norm, finite values)
inline bool is_valid(const geometry_msgs::msg::Quaternion & q)
{
  if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) ||
    !std::isfinite(q.w))
  {
    return false;
  }
  const double norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return norm_sq > 1e-10;
}

// Normalize quaternion
inline geometry_msgs::msg::Quaternion normalize(
  const geometry_msgs::msg::Quaternion & q)
{
  geometry_msgs::msg::Quaternion result;
  const double norm =
    std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (norm > 1e-10) {
    result.x = q.x / norm;
    result.y = q.y / norm;
    result.z = q.z / norm;
    result.w = q.w / norm;
  } else {
    // Return identity if degenerate
    result.x = 0.0;
    result.y = 0.0;
    result.z = 0.0;
    result.w = 1.0;
  }
  return result;
}

// Quaternion conjugate (inverse for unit quaternion)
inline geometry_msgs::msg::Quaternion conjugate(
  const geometry_msgs::msg::Quaternion & q)
{
  geometry_msgs::msg::Quaternion result;
  result.x = -q.x;
  result.y = -q.y;
  result.z = -q.z;
  result.w = q.w;
  return result;
}

// Quaternion multiplication: q1 * q2
inline geometry_msgs::msg::Quaternion multiply(
  const geometry_msgs::msg::Quaternion & q1,
  const geometry_msgs::msg::Quaternion & q2)
{
  geometry_msgs::msg::Quaternion result;
  result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
  result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
  result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
  result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
  return result;
}

// Rotate vector by quaternion: q * [0, v] * q^(-1)
inline geometry_msgs::msg::Vector3 rotate_vector(
  const geometry_msgs::msg::Quaternion & q,
  const geometry_msgs::msg::Vector3 & v)
{
  // Optimized rotation formula (avoiding full quaternion multiplication)
  // v' = v + 2 * q.w * (q_xyz × v) + 2 * (q_xyz × (q_xyz × v))

  const double qx = q.x, qy = q.y, qz = q.z, qw = q.w;
  const double vx = v.x, vy = v.y, vz = v.z;

  // t = 2 * (q_xyz × v)
  const double tx = 2.0 * (qy * vz - qz * vy);
  const double ty = 2.0 * (qz * vx - qx * vz);
  const double tz = 2.0 * (qx * vy - qy * vx);

  geometry_msgs::msg::Vector3 result;
  result.x = vx + qw * tx + (qy * tz - qz * ty);
  result.y = vy + qw * ty + (qz * tx - qx * tz);
  result.z = vz + qw * tz + (qx * ty - qy * tx);

  return result;
}

// Identity quaternion
inline geometry_msgs::msg::Quaternion identity()
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = 0.0;
  q.w = 1.0;
  return q;
}

} // namespace quat_utils

} // namespace eskf_localization

#endif // ESKF_LOCALIZATION__EMA_FILTER_HPP_
