#ifndef ESKF_LOCALIZATION__ESKF_CORE_HPP_
#define ESKF_LOCALIZATION__ESKF_CORE_HPP_

#include <cmath>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace eskf_localization
{

struct EskfCoreParams
{
  // Initial covariance (variance, not stddev)
  double init_pos_var{1.0e6};
  double init_vel_var{1.0e2};
  double init_att_var{(30.0 * 3.14159265358979323846 / 180.0) *
    (30.0 * 3.14159265358979323846 / 180.0)};
  double init_bg_var{(0.1) * (0.1)};
  double init_ba_var{(0.5) * (0.5)};

  // Continuous-time process noise (stddev)
  double gyro_noise_std{0.05};      // rad/s/sqrt(Hz)
  double accel_noise_std{1.0};      // m/s^2/sqrt(Hz)
  double gyro_bias_noise_std{1e-4}; // rad/s^2/sqrt(Hz) (random walk)
  double accel_bias_noise_std{1e-3}; // m/s^3/sqrt(Hz) (random walk)

  // NIS gating thresholds (chi-square). Set <= 0 to disable.
  double nis_gate_gnss_pos_3d{11.34}; // dof=3, ~99%
  double nis_gate_gnss_vel_3d{11.34}; // dof=3, ~99%
  double nis_gate_heading_yaw{6.63};  // dof=1, ~99%

  // If true, inflate R when NIS is too large; else skip update.
  bool nis_gate_inflate{true};
  double nis_gate_inflate_max{10000.0}; // cap for R inflation factor
  // Exponential decay of final R_eff inflate factor across updates.
  // - larger value => slower decay
  // - <=0 disables smoothing
  double r_eff_decay_tau_updates{50.0};

  // Maximum correction allowed before skipping update (to protect reset Jacobian)
  double max_correction_pos_m{50.0};
  double max_correction_vel_mps{20.0};
  double max_correction_att_rad{0.0};
  bool use_so3_jacobian_reset{true};
};

struct EskfGnssPosUpdateDebug
{
  bool applied{false};
  double nis{0.0};
  Eigen::Vector3d residual{Eigen::Vector3d::Zero()};
  std::string reason{};
  Eigen::Vector3d R_diag{Eigen::Vector3d::Zero()};  // Final R diagonal (with all inflates)
};

struct EskfGnssVelUpdateDebug
{
  bool applied{false};
  double nis{0.0};
  Eigen::Vector3d residual{Eigen::Vector3d::Zero()};
  std::string reason{};
  Eigen::Vector3d R_diag{Eigen::Vector3d::Zero()};  // Final R diagonal (with all inflates)
};

struct EskfScalarUpdateDebug
{
  bool applied{false};
  double nis{0.0};
  double residual_rad{0.0};
  std::string reason{};
  double R{0.0};  // Final R value (with all inflates)
};

using EskfYawUpdateDebug = EskfScalarUpdateDebug;

class EskfCore
{
public:
  using P15 = Eigen::Matrix<double, 15, 15>;
  using V15 = Eigen::Matrix<double, 15, 1>;

  explicit EskfCore(const EskfCoreParams & params = EskfCoreParams{});

  void set_params(const EskfCoreParams & params) {m_params = params;}
  const EskfCoreParams & params() const {return m_params;}

  bool initialized() const {return m_initialized;}
  bool finite() const;

  void reset();

  void initialize(
    const Eigen::Vector3d & p_map,
    const Eigen::Quaterniond & q_map_from_base);

  void propagate(
    const Eigen::Vector3d & omega_base_radps,
    const Eigen::Vector3d & accel_base_mps2, double dt_sec);

  EskfGnssPosUpdateDebug update_gnss_position_3d(
    const Eigen::Vector3d & z_p_map,
    const Eigen::Matrix3d & R);

  EskfGnssVelUpdateDebug update_gnss_velocity_3d(
    const Eigen::Vector3d & z_v_map,
    const Eigen::Matrix3d & R);

  EskfYawUpdateDebug update_heading_yaw(double z_yaw_rad, double yaw_var_rad2);

  // Vehicle constraints (Phase 8 add-ons)
  // axis: 0=x, 1=y, 2=z in base frame
  EskfScalarUpdateDebug update_body_velocity_component(
    int axis,
    double z_v_base_mps,
    double var_mps2);

  // Pseudo-measurement to constrain gyro bias using kinematic yaw rate:
  // yaw_rate_ref ≈ v*tan(delta)/L
  EskfScalarUpdateDebug update_yaw_rate_from_steer(
    double omega_meas_z_radps,
    double yaw_rate_ref_radps,
    double var_radps2);

  // State getters
  const Eigen::Vector3d & p_map() const {return m_p_map;}
  const Eigen::Vector3d & v_map() const {return m_v_map;}
  const Eigen::Quaterniond & q_map_from_base() const {return m_q_map_from_base;}
  const Eigen::Vector3d & b_g() const {return m_b_g;}
  const Eigen::Vector3d & b_a() const {return m_b_a;}
  const P15 & P() const {return m_P;}

private:
  enum class RInflateChannel
  {
    kNone,
    kGnssPos,
    kGnssVel,
    kHeadingYaw
  };

  static Eigen::Matrix3d skew(const Eigen::Vector3d & v);
  static Eigen::Quaterniond delta_quat_from_dtheta(const Eigen::Vector3d & dtheta);

  void inject_and_reset(const V15 & dx);
  void symmetrize_P();
  double smooth_r_eff_inflate(
    RInflateChannel channel,
    double target_factor);

  const char * check_large_correction(const V15 & dx) const;

  template<int Dim, typename DebugType>
  DebugType update_vector_measurement(
    const Eigen::Matrix<double, Dim, 15> & H,
    const Eigen::Matrix<double, Dim, 1> & r,
    const Eigen::Matrix<double, Dim, Dim> & R, double gate,
    RInflateChannel channel = RInflateChannel::kNone);

  EskfScalarUpdateDebug update_scalar_measurement(
    const Eigen::Matrix<double, 1, 15> & H, double r, double var,
    double gate,
    RInflateChannel channel = RInflateChannel::kNone);

  EskfCoreParams m_params{};
  bool m_initialized{false};

  // Nominal state
  Eigen::Vector3d m_p_map{Eigen::Vector3d::Zero()};
  Eigen::Vector3d m_v_map{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond m_q_map_from_base{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d m_b_g{Eigen::Vector3d::Zero()};
  Eigen::Vector3d m_b_a{Eigen::Vector3d::Zero()};

  // Error-state covariance
  P15 m_P{P15::Zero()};

  // Final R_eff inflation state (asymmetric: rise-immediate / decay-exponential)
  double m_r_eff_inflate_gnss_pos_{1.0};
  double m_r_eff_inflate_gnss_vel_{1.0};
  double m_r_eff_inflate_heading_yaw_{1.0};
};

} // namespace eskf_localization

#endif // ESKF_LOCALIZATION__ESKF_CORE_HPP_
