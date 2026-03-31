#include "eskf_localization/eskf_localization_node.hpp"

#include <algorithm>
#include <cctype>
#include <limits>

namespace eskf_localization
{

// NOTE: 노드 파라미터 로딩의 단일 진입점. 실제 declare_parameter 로직은
// parameters.cpp에 모아두고 여기서는 적용/반영만 수행한다.
void ESKFLocalizationNode::load_parameters()
{
  m_node_params.load(*this);
  m_node_params.heading_arbitrator.min_status_for_gphdt =
    m_node_params.gnss.min_status_for_yaw_update;
  m_time_processor = TimeProcessor(m_node_params.time);
  m_imu_preprocessor.set_params(m_node_params.imu);
  m_eskf.set_params(m_node_params.eskf);
  {
    std::scoped_lock<std::mutex> lock(m_heading_arbitrator_mutex);
    m_heading_arbitrator.set_params(m_node_params.heading_arbitrator);
    m_heading_arbitrator.reset();
  }
  m_last_yaw_meas_var_rad2 = m_node_params.heading.yaw_var;
  m_last_heading_status_inflate_dbg = 1.0;
  m_last_heading_recover_inflate_dbg = 1.0;
  m_last_heading_yaw_var_pre_nis_dbg = m_last_yaw_meas_var_rad2;
  m_last_heading_yaw_var_applied_dbg = std::numeric_limits<double>::quiet_NaN();
  m_last_heading_yaw_var_source_dbg = "normal";

  // Init/activation mode parsing
  std::string mode = m_node_params.init.mode;
  std::transform(
    mode.begin(), mode.end(), mode.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  m_use_external_initialpose_ = (mode == "external_initialpose");
  if (!(mode == "auto" || mode == "external_initialpose")) {
    RCLCPP_WARN(
      this->get_logger(),
      "Unknown init.mode='%s'. Falling back to 'auto'.",
      m_node_params.init.mode.c_str());
    m_use_external_initialpose_ = false;
  }

  // If trigger is required, start deactivated until pose_initializer (or user) activates.
  m_is_activated_ = !m_node_params.init.require_trigger;
}

} // namespace eskf_localization
