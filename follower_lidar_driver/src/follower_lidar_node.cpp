#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "std_srvs/srv/empty.hpp"

#include "CYdLidar.h"

namespace
{
template<typename T>
T declare_and_get(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name,
  const T & default_value)
{
  node->declare_parameter<T>(name, default_value);
  return node->get_parameter(name).get_value<T>();
}

void set_bool_lidar_option(CYdLidar & laser, int prop, bool value)
{
  laser.setlidaropt(prop, &value, sizeof(bool));
}

void set_int_lidar_option(CYdLidar & laser, int prop, int value)
{
  laser.setlidaropt(prop, &value, sizeof(int));
}

void set_float_lidar_option(CYdLidar & laser, int prop, float value)
{
  laser.setlidaropt(prop, &value, sizeof(float));
}
}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("follower_lidar_node");

  const auto port = declare_and_get<std::string>(node, "port", "/dev/ttyLIDAR");
  const auto frame_id = declare_and_get<std::string>(node, "frame_id", "follower/lidar");
  const auto scan_topic = declare_and_get<std::string>(node, "scan_topic", "/follower/scan");
  const auto publish_point_cloud = declare_and_get<bool>(node, "publish_point_cloud", false);
  const auto point_cloud_topic =
    declare_and_get<std::string>(node, "point_cloud_topic", "/follower/lidar/point_cloud");
  const auto ignore_array = declare_and_get<std::string>(node, "ignore_array", "");

  CYdLidar laser;
  laser.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());
  laser.setlidaropt(LidarPropIgnoreArray, ignore_array.c_str(), ignore_array.size());

  set_int_lidar_option(
    laser, LidarPropSerialBaudrate,
    declare_and_get<int>(node, "baudrate", 512000));
  set_int_lidar_option(
    laser, LidarPropLidarType,
    declare_and_get<int>(node, "lidar_type", TYPE_TRIANGLE));
  set_int_lidar_option(
    laser, LidarPropDeviceType,
    declare_and_get<int>(node, "device_type", YDLIDAR_TYPE_SERIAL));
  set_int_lidar_option(
    laser, LidarPropSampleRate,
    declare_and_get<int>(node, "sample_rate", 4));
  set_int_lidar_option(
    laser, LidarPropAbnormalCheckCount,
    declare_and_get<int>(node, "abnormal_check_count", 4));
  set_int_lidar_option(
    laser, LidarPropIntenstiyBit,
    declare_and_get<int>(node, "intensity_bit", 10));

  set_bool_lidar_option(
    laser, LidarPropFixedResolution,
    declare_and_get<bool>(node, "fixed_resolution", true));
  set_bool_lidar_option(
    laser, LidarPropReversion,
    declare_and_get<bool>(node, "reversion", true));
  set_bool_lidar_option(
    laser, LidarPropInverted,
    declare_and_get<bool>(node, "inverted", true));
  set_bool_lidar_option(
    laser, LidarPropAutoReconnect,
    declare_and_get<bool>(node, "auto_reconnect", true));
  set_bool_lidar_option(
    laser, LidarPropSingleChannel,
    declare_and_get<bool>(node, "is_single_channel", false));
  set_bool_lidar_option(
    laser, LidarPropIntenstiy,
    declare_and_get<bool>(node, "intensity", true));
  set_bool_lidar_option(
    laser, LidarPropSupportMotorDtrCtrl,
    declare_and_get<bool>(node, "support_motor_dtr", false));

  set_float_lidar_option(
    laser, LidarPropMaxAngle,
    static_cast<float>(declare_and_get<double>(node, "angle_max", 180.0)));
  set_float_lidar_option(
    laser, LidarPropMinAngle,
    static_cast<float>(declare_and_get<double>(node, "angle_min", -180.0)));
  set_float_lidar_option(
    laser, LidarPropMaxRange,
    static_cast<float>(declare_and_get<double>(node, "range_max", 16.0)));
  set_float_lidar_option(
    laser, LidarPropMinRange,
    static_cast<float>(declare_and_get<double>(node, "range_min", 0.1)));
  set_float_lidar_option(
    laser, LidarPropScanFrequency,
    static_cast<float>(declare_and_get<double>(node, "frequency", 10.0)));

  const auto loop_rate_hz = declare_and_get<double>(node, "loop_rate_hz", 20.0);
  const auto invalid_range_is_inf =
    declare_and_get<bool>(node, "invalid_range_is_inf", false);

  RCLCPP_INFO(
    node->get_logger(),
    "follower_lidar_node starting: port=%s frame_id=%s scan_topic=%s",
    port.c_str(), frame_id.c_str(), scan_topic.c_str());

  bool ok = laser.initialize();
  if (ok) {
    ok = laser.turnOn();
  }
  if (!ok) {
    RCLCPP_ERROR(node->get_logger(), "YDLidar start failed: %s", laser.DescribeError());
    laser.disconnecting();
    rclcpp::shutdown();
    return 1;
  }

  auto scan_pub =
    node->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic, rclcpp::SensorDataQoS());
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr point_cloud_pub;
  if (publish_point_cloud) {
    point_cloud_pub =
      node->create_publisher<sensor_msgs::msg::PointCloud>(
      point_cloud_topic, rclcpp::SensorDataQoS());
  }

  auto stop_service = node->create_service<std_srvs::srv::Empty>(
    "stop_scan",
    [&laser](
      const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<std_srvs::srv::Empty::Request>,
      std::shared_ptr<std_srvs::srv::Empty::Response>) {
      laser.turnOff();
    });
  auto start_service = node->create_service<std_srvs::srv::Empty>(
    "start_scan",
    [&laser](
      const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<std_srvs::srv::Empty::Request>,
      std::shared_ptr<std_srvs::srv::Empty::Response>) {
      laser.turnOn();
    });

  rclcpp::WallRate loop_rate(loop_rate_hz);
  while (rclcpp::ok()) {
    LaserScan scan;
    if (!laser.doProcessSimple(scan)) {
      RCLCPP_WARN_THROTTLE(
        node->get_logger(), *node->get_clock(), 2000,
        "failed to read YDLidar scan: %s", laser.DescribeError());
      rclcpp::spin_some(node);
      loop_rate.sleep();
      continue;
    }

    sensor_msgs::msg::LaserScan scan_msg;
    scan_msg.header.stamp = node->get_clock()->now();
    scan_msg.header.frame_id = frame_id;
    scan_msg.angle_min = scan.config.min_angle;
    scan_msg.angle_max = scan.config.max_angle;
    scan_msg.angle_increment = scan.config.angle_increment;
    scan_msg.scan_time = scan.config.scan_time;
    scan_msg.time_increment = scan.config.time_increment;
    scan_msg.range_min = scan.config.min_range;
    scan_msg.range_max = scan.config.max_range;

    const auto size =
      static_cast<std::size_t>(
      std::floor(
        (scan.config.max_angle - scan.config.min_angle) /
        scan.config.angle_increment) + 1.0);
    const float invalid_value = invalid_range_is_inf ? INFINITY : 0.0F;
    scan_msg.ranges.assign(size, invalid_value);
    scan_msg.intensities.assign(size, 0.0F);

    sensor_msgs::msg::PointCloud point_cloud_msg;
    if (publish_point_cloud) {
      point_cloud_msg.header = scan_msg.header;
      point_cloud_msg.channels.resize(2);
      point_cloud_msg.channels[0].name = "intensities";
      point_cloud_msg.channels[1].name = "stamps";
    }

    for (std::size_t i = 0; i < scan.points.size(); ++i) {
      const auto & point = scan.points[i];
      const auto index =
        static_cast<int>(
        std::ceil((point.angle - scan.config.min_angle) / scan.config.angle_increment));
      if (index >= 0 && static_cast<std::size_t>(index) < size &&
        point.range >= scan.config.min_range && point.range <= scan.config.max_range)
      {
        scan_msg.ranges[static_cast<std::size_t>(index)] = point.range;
        scan_msg.intensities[static_cast<std::size_t>(index)] = point.intensity;
      }

      if (publish_point_cloud &&
        point.range >= scan.config.min_range && point.range <= scan.config.max_range)
      {
        geometry_msgs::msg::Point32 ros_point;
        ros_point.x = point.range * std::cos(point.angle);
        ros_point.y = point.range * std::sin(point.angle);
        ros_point.z = 0.0F;
        point_cloud_msg.points.push_back(ros_point);
        point_cloud_msg.channels[0].values.push_back(point.intensity);
        point_cloud_msg.channels[1].values.push_back(
          static_cast<float>(i) * scan.config.time_increment);
      }
    }

    scan_pub->publish(scan_msg);
    if (publish_point_cloud && point_cloud_pub) {
      point_cloud_pub->publish(point_cloud_msg);
    }

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  RCLCPP_INFO(node->get_logger(), "stopping YDLidar");
  laser.turnOff();
  laser.disconnecting();
  rclcpp::shutdown();
  return 0;
}
