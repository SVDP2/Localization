#include "eskf_localization/parameters.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cmath>
#include <exception>
#include <string>

namespace eskf_localization
{

// NOTE: 런타임 파라미터 선언/기본값 적용을 한 곳에서 관리한다.
void ESKFLocalizationNodeParams::load(rclcpp::Node & node)
{
  io.imu_topic = node.declare_parameter("imu_topic", io.imu_topic);
  io.gnss_topic = node.declare_parameter("gnss_topic", io.gnss_topic);
  io.gnss_vel_topic = node.declare_parameter("gnss_vel_topic", io.gnss_vel_topic);
  io.heading_topic = node.declare_parameter("heading_topic", io.heading_topic);
  io.map_projector_info_topic = node.declare_parameter(
    "map_projector_info_topic", io.map_projector_info_topic);
  io.velocity_topic = node.declare_parameter("velocity_topic", io.velocity_topic);
  io.steering_topic = node.declare_parameter("steering_topic", io.steering_topic);
  io.output_odom_topic =
    node.declare_parameter("output_odom_topic", io.output_odom_topic);
  io.map_frame = node.declare_parameter("map_frame", io.map_frame);
  io.base_frame = node.declare_parameter("base_frame", io.base_frame);
  io.publish_tf = node.declare_parameter("publish_tf", io.publish_tf);
  io.publish_rate = node.declare_parameter("publish_rate", io.publish_rate);

  const double tol_ms =
    node.declare_parameter("time_alignment_tolerance_ms", 2.0);
  if (std::isfinite(tol_ms) && tol_ms > 0.0) {
    time_alignment_tolerance_sec = tol_ms * 1e-3;
  } else {
    time_alignment_tolerance_sec = 0.0;
  }

  heading.enable_yaw_update =
    node.declare_parameter(
    "heading.enable_yaw_update",
    heading.enable_yaw_update);
  heading.yaw_var = node.declare_parameter("heading.yaw_var", heading.yaw_var);
  heading.max_rate_radps = node.declare_parameter(
    "heading.max_rate_radps",
    heading.max_rate_radps);
  heading.rate_gate_skip_max_count =
    node.declare_parameter(
    "heading.rate_gate_skip_max_count",
    heading.rate_gate_skip_max_count);
  heading.rate_gate_skip_max_sec =
    node.declare_parameter(
    "heading.rate_gate_skip_max_sec",
    heading.rate_gate_skip_max_sec);
  heading.rate_gate_bypass_yaw_var_scale =
    node.declare_parameter(
    "heading.rate_gate_bypass_yaw_var_scale",
    heading.rate_gate_bypass_yaw_var_scale);

  heading_arbitrator.gphdt_recover_holdoff_sec =
    node.declare_parameter(
    "heading.arb.gphdt_recover_holdoff_sec",
    heading_arbitrator.gphdt_recover_holdoff_sec);
  heading_arbitrator.max_time_diff_sec =
    node.declare_parameter(
    "heading.arb.max_time_diff_sec",
    heading_arbitrator.max_time_diff_sec);

  vehicle_constraints.enable_speed_update =
    node.declare_parameter(
    "vehicle.enable_speed_update",
    vehicle_constraints.enable_speed_update);
  vehicle_constraints.speed_use_abs = node.declare_parameter(
    "vehicle.speed_use_abs", vehicle_constraints.speed_use_abs);
  vehicle_constraints.speed_var = node.declare_parameter(
    "vehicle.speed_var", vehicle_constraints.speed_var);
  vehicle_constraints.min_speed_mps_for_speed_update =
    node.declare_parameter(
    "vehicle.min_speed_mps_for_speed_update",
    vehicle_constraints.min_speed_mps_for_speed_update);

  vehicle_constraints.enable_nhc = node.declare_parameter(
    "vehicle.enable_nhc", vehicle_constraints.enable_nhc);
  vehicle_constraints.nhc_var =
    node.declare_parameter("vehicle.nhc_var", vehicle_constraints.nhc_var);

  vehicle_constraints.enable_zupt = node.declare_parameter(
    "vehicle.enable_zupt", vehicle_constraints.enable_zupt);
  vehicle_constraints.zupt_speed_threshold_mps =
    node.declare_parameter(
    "vehicle.zupt_speed_threshold_mps",
    vehicle_constraints.zupt_speed_threshold_mps);
  vehicle_constraints.zupt_var =
    node.declare_parameter("vehicle.zupt_var", vehicle_constraints.zupt_var);

  vehicle_constraints.enable_yaw_rate_update =
    node.declare_parameter(
    "vehicle.enable_yaw_rate_update",
    vehicle_constraints.enable_yaw_rate_update);
  vehicle_constraints.yaw_rate_min_speed_mps = node.declare_parameter(
    "vehicle.yaw_rate_min_speed_mps",
    vehicle_constraints.yaw_rate_min_speed_mps);
  vehicle_constraints.yaw_rate_var = node.declare_parameter(
    "vehicle.yaw_rate_var", vehicle_constraints.yaw_rate_var);

  vehicle.wheelbase = node.declare_parameter("wheelbase", vehicle.wheelbase);

  init_imu_calibration =
    node.declare_parameter("init_imu_calibration", false);

  std::string default_calibration_path;
  try {
    const std::string package_share =
      ament_index_cpp::get_package_share_directory("eskf_localization");
    default_calibration_path = package_share + "/config/imu_calibration.yaml";
  } catch (const std::exception &) {
    default_calibration_path = "config/imu_calibration.yaml";
  }
  calibration_file_path = node.declare_parameter(
    "calibration_file_path",
    default_calibration_path);

  imu.gyro_lpf_cutoff_hz = node.declare_parameter(
    "imu.gyro_lpf_cutoff_hz",
    imu.gyro_lpf_cutoff_hz);
  imu.accel_lpf_cutoff_hz = node.declare_parameter(
    "imu.accel_lpf_cutoff_hz",
    imu.accel_lpf_cutoff_hz);
  imu.gravity_magnitude = node.declare_parameter(
    "imu.gravity_magnitude",
    imu.gravity_magnitude);
  imu.enable_gravity_removal = node.declare_parameter(
    "imu.enable_gravity_removal", imu.enable_gravity_removal);
  imu.enable_orientation_calibration = node.declare_parameter(
    "imu.enable_orientation_calibration", imu.enable_orientation_calibration);

  gnss.enable_gnss_pos_update = node.declare_parameter(
    "enable_gnss_pos_update", gnss.enable_gnss_pos_update);
  gnss.enable_gnss_vel_update = node.declare_parameter(
    "enable_gnss_vel_update", gnss.enable_gnss_vel_update);

  gnss.use_navsatfix_covariance =
    node.declare_parameter(
    "gnss.use_navsatfix_covariance",
    gnss.use_navsatfix_covariance);
  gnss.cov_scale = node.declare_parameter(
    "gnss.covariance_scale",
    gnss.cov_scale);
  gnss.pos_var_fallback = node.declare_parameter(
    "gnss.pos_var_fallback",
    gnss.pos_var_fallback);
  gnss.pos_var_min = node.declare_parameter(
    "gnss.pos_var_min",
    gnss.pos_var_min);
  gnss.pos_var_max = node.declare_parameter(
    "gnss.pos_var_max",
    gnss.pos_var_max);
  gnss.pos_cov_diag_only = node.declare_parameter(
    "gnss.pos_cov_diag_only",
    gnss.pos_cov_diag_only);
  gnss.pos_inflate_status_fix = node.declare_parameter(
    "gnss.pos_inflate_status_fix", gnss.pos_inflate_status_fix);
  gnss.pos_inflate_status_sbas = node.declare_parameter(
    "gnss.pos_inflate_status_sbas", gnss.pos_inflate_status_sbas);
  gnss.vel_cov_scale = node.declare_parameter(
    "gnss.vel_covariance_scale",
    gnss.vel_cov_scale);
  gnss.vel_var_fallback = node.declare_parameter(
    "gnss.vel_var_fallback",
    gnss.vel_var_fallback);
  gnss.vel_var_min = node.declare_parameter(
    "gnss.vel_var_min",
    gnss.vel_var_min);
  gnss.vel_var_max = node.declare_parameter(
    "gnss.vel_var_max",
    gnss.vel_var_max);

  gnss.min_status_for_pos_update = node.declare_parameter(
    "gnss.min_status_for_pos_update", gnss.min_status_for_pos_update);
  gnss.min_status_for_yaw_update = node.declare_parameter(
    "heading.min_gnss_status_for_yaw_update", gnss.min_status_for_yaw_update);
  gnss.vel_inflate_status_fix = node.declare_parameter(
    "gnss.vel_inflate_status_fix", gnss.vel_inflate_status_fix);
  gnss.vel_inflate_status_sbas = node.declare_parameter(
    "gnss.vel_inflate_status_sbas", gnss.vel_inflate_status_sbas);

  // GNSS recovery transition (status upgrade smoothing)
  gnss.recover.enable = node.declare_parameter(
    "gnss.recover.enable", gnss.recover.enable);
  gnss.recover.holdoff_sec = node.declare_parameter(
    "gnss.recover.holdoff_sec", gnss.recover.holdoff_sec);
  gnss.recover.ramp_sec = node.declare_parameter(
    "gnss.recover.ramp_sec", gnss.recover.ramp_sec);
  gnss.recover.pos_max_inflate = node.declare_parameter(
    "gnss.recover.pos_max_inflate", gnss.recover.pos_max_inflate);
  gnss.recover.vel_max_inflate = node.declare_parameter(
    "gnss.recover.vel_max_inflate", gnss.recover.vel_max_inflate);

  // Optional GNSS pose output for pose_initializer compatibility
  gnss.publish_pose_with_covariance = node.declare_parameter(
    "gnss.publish_pose_with_covariance", gnss.publish_pose_with_covariance);
  gnss.pose_with_covariance_topic = node.declare_parameter(
    "gnss.pose_with_covariance_topic", gnss.pose_with_covariance_topic);
  gnss.pose_with_covariance_use_base_position = node.declare_parameter(
    "gnss.pose_with_covariance_use_base_position",
    gnss.pose_with_covariance_use_base_position);

  output.flatten_roll_pitch = node.declare_parameter(
    "output.flatten_roll_pitch", output.flatten_roll_pitch);
  output.roll_pitch_var =
    node.declare_parameter("output.roll_pitch_var", output.roll_pitch_var);

  // Localization initialization interface (Autoware-compatible)
  init.mode = node.declare_parameter("init.mode", init.mode);
  init.require_trigger =
    node.declare_parameter("init.require_trigger", init.require_trigger);
  init.external_initialpose_topic = node.declare_parameter(
    "init.external_initialpose_topic", init.external_initialpose_topic);
  init.reset_on_external_pose = node.declare_parameter(
    "init.reset_on_external_pose", init.reset_on_external_pose);

  // KISS-ICP tight coupling (Phase 2)
  kiss_icp.enable = node.declare_parameter("kiss_icp.enable", kiss_icp.enable);
  kiss_icp.pointcloud_topic =
    node.declare_parameter("kiss_icp.pointcloud_topic", kiss_icp.pointcloud_topic);
  kiss_icp.publish_debug_tf = node.declare_parameter(
    "kiss_icp.publish_debug_tf",
    kiss_icp.publish_debug_tf);
  kiss_icp.debug_child_frame = node.declare_parameter(
    "kiss_icp.debug_child_frame", kiss_icp.debug_child_frame);

  kiss_icp.deskew = node.declare_parameter("kiss_icp.data.deskew", kiss_icp.deskew);
  kiss_icp.max_range =
    node.declare_parameter("kiss_icp.data.max_range", kiss_icp.max_range);
  kiss_icp.min_range =
    node.declare_parameter("kiss_icp.data.min_range", kiss_icp.min_range);
  kiss_icp.voxel_size =
    node.declare_parameter("kiss_icp.mapping.voxel_size", kiss_icp.voxel_size);
  kiss_icp.max_points_per_voxel = node.declare_parameter(
    "kiss_icp.mapping.max_points_per_voxel", kiss_icp.max_points_per_voxel);
  kiss_icp.max_num_iterations = node.declare_parameter(
    "kiss_icp.registration.max_num_iterations", kiss_icp.max_num_iterations);
  kiss_icp.convergence_criterion = node.declare_parameter(
    "kiss_icp.registration.convergence_criterion", kiss_icp.convergence_criterion);
  kiss_icp.max_num_threads = node.declare_parameter(
    "kiss_icp.registration.max_num_threads", kiss_icp.max_num_threads);

  kiss_icp.enable_yaw_update = node.declare_parameter(
    "kiss_icp.enable_yaw_update", kiss_icp.enable_yaw_update);
  kiss_icp.enable_vy_update = node.declare_parameter(
    "kiss_icp.enable_vy_update", kiss_icp.enable_vy_update);

  kiss_icp.yaw_var = node.declare_parameter("kiss_icp.yaw_var", kiss_icp.yaw_var);
  kiss_icp.vy_var = node.declare_parameter("kiss_icp.vy_var", kiss_icp.vy_var);

  kiss_icp.min_source_points = node.declare_parameter(
    "kiss_icp.min_source_points", kiss_icp.min_source_points);
  kiss_icp.max_abs_yaw_rate_radps = node.declare_parameter(
    "kiss_icp.max_abs_yaw_rate_radps", kiss_icp.max_abs_yaw_rate_radps);
  kiss_icp.max_abs_vy_mps =
    node.declare_parameter("kiss_icp.max_abs_vy_mps", kiss_icp.max_abs_vy_mps);

  const double icp_tol_ms =
    node.declare_parameter(
    "kiss_icp.time_alignment_tolerance_ms",
    kiss_icp.time_alignment_tolerance_sec * 1e3);
  if (std::isfinite(icp_tol_ms) && icp_tol_ms > 0.0) {
    kiss_icp.time_alignment_tolerance_sec = icp_tol_ms * 1e-3;
  } else {
    kiss_icp.time_alignment_tolerance_sec = 0.0;
  }

  kiss_icp.trust_tau_sec =
    node.declare_parameter("kiss_icp.trust_tau_sec", kiss_icp.trust_tau_sec);
  kiss_icp.trust_status_neg = node.declare_parameter(
    "kiss_icp.trust_status_neg", kiss_icp.trust_status_neg);
  kiss_icp.trust_status_0 =
    node.declare_parameter("kiss_icp.trust_status_0", kiss_icp.trust_status_0);
  kiss_icp.trust_status_1 =
    node.declare_parameter("kiss_icp.trust_status_1", kiss_icp.trust_status_1);
  kiss_icp.trust_status_2 =
    node.declare_parameter("kiss_icp.trust_status_2", kiss_icp.trust_status_2);

  kiss_icp.enable_auto_reset = node.declare_parameter(
    "kiss_icp.enable_auto_reset", kiss_icp.enable_auto_reset);
  kiss_icp.auto_reset_err_pos_m = node.declare_parameter(
    "kiss_icp.auto_reset_err_pos_m", kiss_icp.auto_reset_err_pos_m);
  kiss_icp.auto_reset_err_yaw_rad = node.declare_parameter(
    "kiss_icp.auto_reset_err_yaw_rad", kiss_icp.auto_reset_err_yaw_rad);
  kiss_icp.auto_reset_hold_sec = node.declare_parameter(
    "kiss_icp.auto_reset_hold_sec", kiss_icp.auto_reset_hold_sec);

  eskf.init_pos_var =
    node.declare_parameter("eskf.init_pos_var", eskf.init_pos_var);
  eskf.init_vel_var =
    node.declare_parameter("eskf.init_vel_var", eskf.init_vel_var);
  eskf.init_att_var =
    node.declare_parameter("eskf.init_att_var", eskf.init_att_var);
  eskf.init_bg_var =
    node.declare_parameter("eskf.init_bg_var", eskf.init_bg_var);
  eskf.init_ba_var =
    node.declare_parameter("eskf.init_ba_var", eskf.init_ba_var);

  eskf.gyro_noise_std = node.declare_parameter(
    "eskf.gyro_noise_std",
    eskf.gyro_noise_std);
  eskf.accel_noise_std = node.declare_parameter(
    "eskf.accel_noise_std",
    eskf.accel_noise_std);
  eskf.gyro_bias_noise_std = node.declare_parameter(
    "eskf.gyro_bias_noise_std",
    eskf.gyro_bias_noise_std);
  eskf.accel_bias_noise_std = node.declare_parameter(
    "eskf.accel_bias_noise_std", eskf.accel_bias_noise_std);

  eskf.nis_gate_gnss_pos_3d = node.declare_parameter(
    "eskf.nis_gate_gnss_pos_3d", eskf.nis_gate_gnss_pos_3d);
  eskf.nis_gate_gnss_vel_3d = node.declare_parameter(
    "eskf.nis_gate_gnss_vel_3d", eskf.nis_gate_gnss_vel_3d);
  eskf.nis_gate_heading_yaw = node.declare_parameter(
    "eskf.nis_gate_heading_yaw", eskf.nis_gate_heading_yaw);
  eskf.nis_gate_inflate =
    node.declare_parameter("eskf.nis_gate_inflate", eskf.nis_gate_inflate);
  eskf.nis_gate_inflate_max = node.declare_parameter(
    "eskf.nis_gate_inflate_max", eskf.nis_gate_inflate_max);
  eskf.r_eff_decay_tau_updates = node.declare_parameter(
    "eskf.r_eff_decay_tau_updates", eskf.r_eff_decay_tau_updates);
  eskf.max_correction_pos_m = node.declare_parameter(
    "eskf.max_correction_pos_m", eskf.max_correction_pos_m);
  eskf.max_correction_vel_mps = node.declare_parameter(
    "eskf.max_correction_vel_mps", eskf.max_correction_vel_mps);
  eskf.max_correction_att_rad = node.declare_parameter(
    "eskf.max_correction_att_rad", eskf.max_correction_att_rad);
  eskf.use_so3_jacobian_reset = node.declare_parameter(
    "eskf.use_so3_jacobian_reset", eskf.use_so3_jacobian_reset);
}

} // namespace eskf_localization
