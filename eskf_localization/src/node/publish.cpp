#include "eskf_localization/eskf_localization_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <Eigen/Eigenvalues>

namespace eskf_localization
{

// NOTE: 출력/진단은 고정 주기의 타이머에서 수행하여 시간 일관성을 유지한다.
void ESKFLocalizationNode::publish_timer_callback()
{
  const bool activated =
    (!m_node_params.init.require_trigger) || m_is_activated_;

  const rclcpp::Time odom_stamp =
    (m_state_stamp.seconds() > 0.0) ? m_state_stamp : this->now();

  bool eskf_initialized = false;
  bool heading_received = false;
  Eigen::Vector3d p_map = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_map = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q_map_from_base = Eigen::Quaterniond::Identity();
  EskfCore::P15 P = EskfCore::P15::Zero();
  geometry_msgs::msg::Vector3 omega_base{};
  bool have_omega = false;

  {
    std::scoped_lock<std::mutex> lock(m_state_mutex);
    if (m_eskf.initialized() && !m_eskf.finite()) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(),
        *this->get_clock(), 2000, "ESKF state became non-finite. Resetting filter.");
      m_eskf.reset();
      m_last_gnss_pos_update_dbg = EskfGnssPosUpdateDebug{};
      m_last_gnss_vel_update_dbg = EskfGnssVelUpdateDebug{};
    }
    eskf_initialized = m_eskf.initialized();
    if (eskf_initialized) {
      p_map = m_eskf.p_map();
      v_map = m_eskf.v_map();
      q_map_from_base = m_eskf.q_map_from_base();
      P = m_eskf.P();
    }
    omega_base = m_last_omega_base;
    have_omega = m_have_last_omega_base;
  }
  heading_received = m_heading_received;

  const bool publish_ready =
    activated && eskf_initialized && heading_received;

  if (activated && !publish_ready) {
    const char * reason = !eskf_initialized ? "waiting_for_eskf_init" :
      (!heading_received ? "waiting_for_heading" : "not_ready");
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Output gated (%s): suppressing /localization/kinematic_state and TF", reason);
  }

  Eigen::Quaterniond q_map_from_base_out = q_map_from_base;
  // 출력용 자세: 평면 주행 모드면 yaw-only로 단순화
  if (eskf_initialized && m_node_params.output.flatten_roll_pitch) {
    const Eigen::Matrix3d R = q_map_from_base.toRotationMatrix();
    const double yaw = std::atan2(R(1, 0), R(0, 0));
    q_map_from_base_out =
      Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  }

  nav_msgs::msg::Odometry odom_msg =
    eskf_initialized ?
    m_odom_builder.build_initialized(
    odom_stamp, p_map, v_map,
    q_map_from_base_out, P, omega_base,
    have_omega) :
    m_odom_builder.build_uninitialized(odom_stamp);

  if (eskf_initialized && m_node_params.output.flatten_roll_pitch) {
    double var = m_node_params.output.roll_pitch_var;
    if (!std::isfinite(var) || !(var > 0.0)) {
      var = 1000.0;
    }
    for (int c = 0; c < 6; ++c) {
      if (c != 3) {
        odom_msg.pose.covariance[3 * 6 + c] = 0.0;
        odom_msg.pose.covariance[c * 6 + 3] = 0.0;
      }
      if (c != 4) {
        odom_msg.pose.covariance[4 * 6 + c] = 0.0;
        odom_msg.pose.covariance[c * 6 + 4] = 0.0;
      }
    }
    odom_msg.pose.covariance[3 * 6 + 3] = var;
    odom_msg.pose.covariance[4 * 6 + 4] = var;
  }

  if (publish_ready && m_node_params.io.publish_tf &&
    m_tf_broadcaster)
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    m_odom_builder.build_tf(odom_msg, tf_msg);
    m_tf_broadcaster->sendTransform(tf_msg);
  }

  if (publish_ready) {
    m_odom_pub->publish(odom_msg);
    if (eskf_initialized && m_pose_pub) {
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header = odom_msg.header;
      pose_msg.pose = odom_msg.pose.pose;
      m_pose_pub->publish(pose_msg);
    }
  }

  // 진단은 주기 다운샘플링 (기본 10회마다 1회)
  if (++m_diag_counter >= 10) {
    m_diag_counter = 0;

    EskfDiagnosticsInput diag_input;
    diag_input.imu_count = m_imu_count;
    diag_input.gnss_count = m_gnss_count;
    diag_input.gnss_vel_count = m_gnss_vel_count;
    diag_input.velocity_count = m_velocity_count;
    diag_input.steering_count = m_steering_count;
    diag_input.is_activated = activated;
    diag_input.eskf_initialized = eskf_initialized;

    diag_input.has_vehicle_R = true;
    diag_input.vehicle_speed_var = m_node_params.vehicle_constraints.speed_var;
    diag_input.nhc_var = m_node_params.vehicle_constraints.nhc_var;
    diag_input.zupt_var = m_node_params.vehicle_constraints.zupt_var;
    diag_input.yaw_rate_var = m_node_params.vehicle_constraints.yaw_rate_var;

    diag_input.has_imu_Q = true;
    diag_input.imu_gyro_noise_std = m_node_params.eskf.gyro_noise_std;
    diag_input.imu_accel_noise_std = m_node_params.eskf.accel_noise_std;
    diag_input.imu_gyro_bias_noise_std =
      m_node_params.eskf.gyro_bias_noise_std;
    diag_input.imu_accel_bias_noise_std =
      m_node_params.eskf.accel_bias_noise_std;

    {
      std::scoped_lock<std::mutex> kiss_diag_lock(m_kiss_diag_mutex_);
      diag_input.has_kiss = true;
      diag_input.kiss_enabled = m_kiss_diag_.enabled;
      diag_input.kiss_initialized = m_kiss_diag_.initialized;
      diag_input.kiss_yaw_enabled = m_kiss_diag_.yaw_enabled;
      diag_input.kiss_vy_enabled = m_kiss_diag_.vy_enabled;
      diag_input.kiss_source_points = m_kiss_diag_.source_points;
      diag_input.kiss_dt_ms = m_kiss_diag_.dt_ms;
      diag_input.kiss_trust = m_kiss_diag_.trust;
      diag_input.kiss_target_trust = m_kiss_diag_.target_trust;
      diag_input.kiss_yaw_rate_radps = m_kiss_diag_.yaw_rate_radps;
      diag_input.kiss_vy_mps = m_kiss_diag_.vy_mps;
      diag_input.kiss_yaw_var_eff = m_kiss_diag_.yaw_var_eff;
      diag_input.kiss_vy_var_eff = m_kiss_diag_.vy_var_eff;
      diag_input.kiss_yaw_applied = m_kiss_diag_.yaw_applied;
      diag_input.kiss_yaw_reason = m_kiss_diag_.yaw_reason;
      diag_input.kiss_yaw_nis = m_kiss_diag_.yaw_nis;
      diag_input.kiss_yaw_R = m_kiss_diag_.yaw_R;
      diag_input.kiss_vy_applied = m_kiss_diag_.vy_applied;
      diag_input.kiss_vy_reason = m_kiss_diag_.vy_reason;
      diag_input.kiss_vy_nis = m_kiss_diag_.vy_nis;
      diag_input.kiss_vy_R = m_kiss_diag_.vy_R;
      diag_input.kiss_skip_reason = m_kiss_diag_.skip_reason;
      diag_input.kiss_time_alignment_error_ms =
        m_kiss_diag_.time_alignment_error_ms;
      diag_input.kiss_reset_candidate = m_kiss_diag_.reset_candidate;
      diag_input.kiss_reset_count = m_kiss_diag_.reset_count;
    }

    const auto latest_gnss = m_latest_gnss;
    if (latest_gnss) {
      diag_input.has_gnss_status = true;
      diag_input.gnss_status = static_cast<int>(latest_gnss->status.status);
    }

    // NIS 기반 R 인플레이트 계수 계산
    const auto calc_nis_inflate = [this](const std::string & reason,
        const double nis,
        const double gate) -> double {
        if (reason != "nis_inflated") {
          return 1.0;
        }
        if (!std::isfinite(nis) || !std::isfinite(gate) || !(gate > 0.0)) {
          return 1.0;
        }
        double factor = nis / gate;
        if (!std::isfinite(factor)) {
          return 1.0;
        }
        if (factor < 1.0) {
          factor = 1.0;
        }
        double cap = std::max(1.0, m_node_params.eskf.nis_gate_inflate_max);
        if (!std::isfinite(cap)) {
          cap = 1.0;
        }
        if (factor > cap) {
          factor = cap;
        }
        return factor;
      };

    double pos_recover_inflate_for_diag = 1.0;
    double vel_recover_inflate_for_diag = 1.0;
    double heading_status_inflate_for_diag = 1.0;
    double heading_recover_inflate_for_diag = 1.0;
    double heading_var_pre_nis_for_diag = m_node_params.heading.yaw_var;
    double heading_var_applied_for_diag = std::numeric_limits<double>::quiet_NaN();
    std::string heading_var_source_for_diag = "normal";

    {
      std::scoped_lock<std::mutex> lock(m_state_mutex);
      diag_input.gnss_pos_update = m_last_gnss_pos_update_dbg;
      diag_input.gnss_vel_update = m_last_gnss_vel_update_dbg;
      diag_input.heading_yaw_update = m_last_heading_yaw_update_dbg;
      pos_recover_inflate_for_diag = m_last_gnss_pos_recover_inflate_dbg;
      vel_recover_inflate_for_diag = m_last_gnss_vel_recover_inflate_dbg;
      heading_status_inflate_for_diag = m_last_heading_status_inflate_dbg;
      heading_recover_inflate_for_diag = m_last_heading_recover_inflate_dbg;
      heading_var_pre_nis_for_diag = m_last_heading_yaw_var_pre_nis_dbg;
      heading_var_applied_for_diag = m_last_heading_yaw_var_applied_dbg;
      heading_var_source_for_diag = m_last_heading_yaw_var_source_dbg;
      if (!std::isfinite(pos_recover_inflate_for_diag) || !(pos_recover_inflate_for_diag > 0.0)) {
        pos_recover_inflate_for_diag = 1.0;
      }
      if (!std::isfinite(vel_recover_inflate_for_diag) || !(vel_recover_inflate_for_diag > 0.0)) {
        vel_recover_inflate_for_diag = 1.0;
      }
      if (!std::isfinite(heading_status_inflate_for_diag) || !(heading_status_inflate_for_diag > 0.0)) {
        heading_status_inflate_for_diag = 1.0;
      }
      if (!std::isfinite(heading_recover_inflate_for_diag) || !(heading_recover_inflate_for_diag > 0.0)) {
        heading_recover_inflate_for_diag = 1.0;
      }
      if (!std::isfinite(heading_var_pre_nis_for_diag) || !(heading_var_pre_nis_for_diag > 0.0)) {
        heading_var_pre_nis_for_diag = std::isfinite(m_last_yaw_meas_var_rad2) ?
          m_last_yaw_meas_var_rad2 :
          m_node_params.heading.yaw_var;
      }
      if (!std::isfinite(heading_var_pre_nis_for_diag) || !(heading_var_pre_nis_for_diag > 0.0)) {
        heading_var_pre_nis_for_diag = 0.01;
      }
      if (heading_var_source_for_diag.empty()) {
        heading_var_source_for_diag = "normal";
      }

      if (eskf_initialized) {
        const auto P_now = m_eskf.P();
        const auto P_sym = 0.5 * (P_now + P_now.transpose());
        const auto diag = P_sym.diagonal();

        diag_input.has_P_stats = true;
        diag_input.P_trace = P_sym.trace();
        diag_input.P_max_diag = diag.maxCoeff();
        diag_input.P_min_diag = diag.minCoeff();

        diag_input.P_pos_max_diag =
          P_sym.block<3, 3>(0, 0).diagonal().maxCoeff();
        diag_input.P_vel_max_diag =
          P_sym.block<3, 3>(3, 3).diagonal().maxCoeff();
        diag_input.P_att_max_diag =
          P_sym.block<3, 3>(6, 6).diagonal().maxCoeff();
        diag_input.P_bg_max_diag =
          P_sym.block<3, 3>(9, 9).diagonal().maxCoeff();
        diag_input.P_ba_max_diag =
          P_sym.block<3, 3>(12, 12).diagonal().maxCoeff();

        Eigen::SelfAdjointEigenSolver<EskfCore::P15> eig(P_sym);
        if (eig.info() == Eigen::Success) {
          diag_input.P_min_eig = eig.eigenvalues().minCoeff();
        } else {
          diag_input.P_min_eig = std::numeric_limits<double>::quiet_NaN();
        }
      }
    }

    if (latest_gnss) {
      {
        double status_inflate = 1.0;
        if (diag_input.gnss_status == 0) {
          status_inflate = m_node_params.gnss.pos_inflate_status_fix;
        } else if (diag_input.gnss_status == 1) {
          status_inflate = m_node_params.gnss.pos_inflate_status_sbas;
        } else {
          status_inflate = 1.0;
        }
        if (!std::isfinite(status_inflate) || !(status_inflate > 0.0)) {
          status_inflate = 1.0;
        }

        // Use final R_diag from update if applied, otherwise compute for reference
        if (diag_input.gnss_pos_update.applied &&
            diag_input.gnss_pos_update.R_diag.allFinite()) {
          diag_input.has_gnss_pos_R = true;
          diag_input.gnss_pos_R_xx = diag_input.gnss_pos_update.R_diag(0);
          diag_input.gnss_pos_R_yy = diag_input.gnss_pos_update.R_diag(1);
          diag_input.gnss_pos_R_zz = diag_input.gnss_pos_update.R_diag(2);
        } else {
          GnssPosUpdateConfig config;
          config.use_navsatfix_covariance =
            m_node_params.gnss.use_navsatfix_covariance;
          config.covariance_scale =
            m_node_params.gnss.cov_scale *
            pos_recover_inflate_for_diag;
          config.pos_var_fallback = m_node_params.gnss.pos_var_fallback;
          config.pos_var_min = m_node_params.gnss.pos_var_min;
          config.pos_var_max = m_node_params.gnss.pos_var_max;
          config.cov_diag_only = m_node_params.gnss.pos_cov_diag_only;

          const Eigen::Matrix3d Rpos =
            m_gnss_update_handler.compute_position_covariance(
            *latest_gnss,
            config);
          diag_input.has_gnss_pos_R = true;
          diag_input.gnss_pos_R_xx = Rpos(0, 0);
          diag_input.gnss_pos_R_yy = Rpos(1, 1);
          diag_input.gnss_pos_R_zz = Rpos(2, 2);
        }
        diag_input.gnss_pos_status_inflate = status_inflate;

        const double gate = m_node_params.eskf.nis_gate_gnss_pos_3d;
        diag_input.gnss_pos_nis_inflate =
          calc_nis_inflate(
          diag_input.gnss_pos_update.reason,
          diag_input.gnss_pos_update.nis, gate);
      }

      {
        double status_inflate = 1.0;
        if (diag_input.gnss_status == 0) {
          status_inflate = m_node_params.gnss.vel_inflate_status_fix;
        } else if (diag_input.gnss_status == 1) {
          status_inflate = m_node_params.gnss.vel_inflate_status_sbas;
        } else {
          status_inflate = 1.0;
        }
        if (!std::isfinite(status_inflate) || !(status_inflate > 0.0)) {
          status_inflate = 1.0;
        }

        // Use final R_diag from update if applied, otherwise compute for reference
        if (diag_input.gnss_vel_update.applied &&
            diag_input.gnss_vel_update.R_diag.allFinite()) {
          diag_input.has_gnss_vel_R = true;
          diag_input.gnss_vel_R_xx = diag_input.gnss_vel_update.R_diag(0);
          diag_input.gnss_vel_R_yy = diag_input.gnss_vel_update.R_diag(1);
          diag_input.gnss_vel_R_zz = diag_input.gnss_vel_update.R_diag(2);
        } else {
          GnssVelUpdateConfig config;
          config.covariance_scale =
            m_node_params.gnss.vel_cov_scale *
            vel_recover_inflate_for_diag;
          config.vel_var_fallback = m_node_params.gnss.vel_var_fallback;
          config.vel_var_min = m_node_params.gnss.vel_var_min;
          config.vel_var_max = m_node_params.gnss.vel_var_max;

          const Eigen::Matrix3d Rvel =
            m_gnss_update_handler.compute_velocity_covariance(config);
          diag_input.has_gnss_vel_R = true;
          diag_input.gnss_vel_R_xx = Rvel(0, 0);
          diag_input.gnss_vel_R_yy = Rvel(1, 1);
          diag_input.gnss_vel_R_zz = Rvel(2, 2);
        }
        diag_input.gnss_vel_status_inflate = status_inflate;

        const double gate = m_node_params.eskf.nis_gate_gnss_vel_3d;
        diag_input.gnss_vel_nis_inflate =
          calc_nis_inflate(
          diag_input.gnss_vel_update.reason,
          diag_input.gnss_vel_update.nis, gate);
      }
    }

    diag_input.has_heading_yaw_R = true;
    // heading_yaw_var: pre-NIS measurement variance that will be fed to ESKF.
    diag_input.heading_yaw_var = heading_var_pre_nis_for_diag;
    diag_input.heading_status_inflate = heading_status_inflate_for_diag;
    diag_input.heading_recover_inflate = heading_recover_inflate_for_diag;
    diag_input.heading_yaw_var_source = heading_var_source_for_diag;
    diag_input.heading_yaw_var_applied = heading_var_applied_for_diag;
    if (diag_input.heading_yaw_update.applied &&
      std::isfinite(diag_input.heading_yaw_update.R) &&
      (diag_input.heading_yaw_update.R > 0.0))
    {
      diag_input.heading_yaw_var_eff = diag_input.heading_yaw_update.R;
      diag_input.heading_yaw_var_applied = diag_input.heading_yaw_update.R;
      if (diag_input.heading_yaw_var > 0.0) {
        diag_input.heading_yaw_nis_inflate =
          diag_input.heading_yaw_var_eff / diag_input.heading_yaw_var;
      } else {
        diag_input.heading_yaw_nis_inflate = 1.0;
      }
    } else {
      diag_input.heading_yaw_nis_inflate = 1.0;
      diag_input.heading_yaw_var_eff = diag_input.heading_yaw_var;
      diag_input.heading_yaw_var_applied = std::numeric_limits<double>::quiet_NaN();
    }

    if (m_imu_dt_stats.has_samples()) {
      diag_input.has_imu_dt_stats = true;
      diag_input.imu_dt_stats = m_imu_dt_stats;
      reset_imu_dt_stats();
    }

    const rclcpp::Time now = this->now();
    if (m_latest_gnss) {
      diag_input.has_gnss_delay = true;
      diag_input.gnss_delay_sec = (now - m_last_gnss_stamp).seconds();
    }
    if (m_latest_gnss_vel) {
      diag_input.has_gnss_vel_delay = true;
      diag_input.gnss_vel_delay_sec = (now - m_last_gnss_vel_stamp).seconds();
    }
    if (m_latest_velocity) {
      diag_input.has_velocity_delay = true;
      diag_input.velocity_delay_sec = (now - m_last_velocity_stamp).seconds();
    }
    if (m_latest_steering) {
      diag_input.has_steering_delay = true;
      diag_input.steering_delay_sec = (now - m_last_steering_stamp).seconds();
    }

    const auto diag_array = m_diag_builder.build(now, diag_input);
    m_diag_pub->publish(diag_array);
  }
}

} // namespace eskf_localization
