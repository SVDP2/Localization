#include "relative_localization_eskf/geometry.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

namespace relative_localization_eskf
{

namespace
{

geometry_msgs::msg::Point point(double x, double y, double z)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

std_msgs::msg::ColorRGBA color(double r, double g, double b, double a)
{
  std_msgs::msg::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}

double rad_to_deg(double rad)
{
  return rad * 180.0 / M_PI;
}

}  // namespace

class PoseRvizMarkerNode : public rclcpp::Node
{
public:
  PoseRvizMarkerNode()
  : Node("pose_rviz_marker_node")
  {
    declare_parameter("odom_topic", "localization/leader_base/odom");
    declare_parameter("marker_topic", "localization/relative/pose_markers");
    declare_parameter("marker_namespace", "relative_pose_viz");
    declare_parameter("axis_line_width", 0.01);
    declare_parameter("text_size", 0.08);
    declare_parameter("vehicle_arrow_length", 0.22);
    declare_parameter("vehicle_arrow_width", 0.05);
    declare_parameter("good_cov_trace", 0.02);
    declare_parameter("warn_cov_trace", 0.12);

    marker_namespace_ = get_parameter("marker_namespace").as_string();
    axis_line_width_ = get_parameter("axis_line_width").as_double();
    text_size_ = get_parameter("text_size").as_double();
    vehicle_arrow_length_ = get_parameter("vehicle_arrow_length").as_double();
    vehicle_arrow_width_ = get_parameter("vehicle_arrow_width").as_double();
    good_cov_trace_ = get_parameter("good_cov_trace").as_double();
    warn_cov_trace_ = get_parameter("warn_cov_trace").as_double();

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      get_parameter("marker_topic").as_string(), 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("odom_topic").as_string(), 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {odom_callback(msg);});

    RCLCPP_INFO(get_logger(), "marker topic: %s", get_parameter("marker_topic").as_string().c_str());
  }

private:
  std_msgs::msg::ColorRGBA color_from_covariance(double trace) const
  {
    if (trace <= good_cov_trace_) {
      return color(0.20, 0.85, 0.36, 1.0);
    }
    if (trace <= warn_cov_trace_) {
      return color(0.95, 0.75, 0.18, 1.0);
    }
    return color(0.95, 0.25, 0.25, 1.0);
  }

  visualization_msgs::msg::Marker base_marker(
    const nav_msgs::msg::Odometry & msg,
    int id,
    int type,
    const std_msgs::msg::ColorRGBA & c) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header = msg.header;
    marker.ns = marker_namespace_;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.color = c;
    marker.lifetime.sec = 0;
    marker.lifetime.nanosec = 500000000;
    return marker;
  }

  visualization_msgs::msg::Marker text_marker(
    const nav_msgs::msg::Odometry & msg,
    int id,
    const std::string & text,
    const geometry_msgs::msg::Point & p,
    const std_msgs::msg::ColorRGBA & c) const
  {
    auto marker = base_marker(msg, id, visualization_msgs::msg::Marker::TEXT_VIEW_FACING, c);
    marker.pose.position = p;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = text_size_;
    marker.text = text;
    return marker;
  }

  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    const auto & p = msg->pose.pose.position;
    const auto & q_msg = msg->pose.pose.orientation;
    const Eigen::Quaterniond q = normalize_quaternion(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
    const Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
    const double yaw_deg = rad_to_deg(yaw_from_quaternion(q));
    const double pitch_deg = rad_to_deg(euler[1]);
    const double roll_deg = rad_to_deg(euler[2]);

    const double cov_trace =
      msg->pose.covariance[0] + msg->pose.covariance[7] + msg->pose.covariance[14];
    const auto c = color_from_covariance(cov_trace);

    visualization_msgs::msg::MarkerArray array;

    auto axis = base_marker(*msg, 0, visualization_msgs::msg::Marker::LINE_LIST, c);
    axis.scale.x = axis_line_width_;
    axis.points = {
      point(0.0, 0.0, 0.0), point(p.x, 0.0, 0.0),
      point(0.0, 0.0, 0.0), point(0.0, p.y, 0.0),
      point(0.0, 0.0, 0.0), point(0.0, 0.0, p.z),
    };
    array.markers.push_back(axis);

    auto arrow = base_marker(*msg, 1, visualization_msgs::msg::Marker::ARROW, c);
    arrow.pose = msg->pose.pose;
    arrow.scale.x = vehicle_arrow_length_;
    arrow.scale.y = vehicle_arrow_width_;
    arrow.scale.z = vehicle_arrow_width_;
    array.markers.push_back(arrow);

    array.markers.push_back(text_marker(*msg, 10, "x " + signed_number(p.x, 3) + " m", point(p.x * 0.5, 0.0, 0.03), c));
    array.markers.push_back(text_marker(*msg, 11, "y " + signed_number(p.y, 3) + " m", point(0.0, p.y * 0.5, 0.08), c));
    array.markers.push_back(text_marker(*msg, 12, "z " + signed_number(p.z, 3) + " m", point(0.0, 0.0, std::max(p.z * 0.5, 0.12)), c));
    array.markers.push_back(text_marker(*msg, 13, "frame " + msg->header.frame_id, point(0.0, 0.0, 0.22), c));
    array.markers.push_back(text_marker(*msg, 14, "roll " + signed_number(roll_deg, 1) + " deg", point(p.x, p.y, p.z + 0.12), c));
    array.markers.push_back(text_marker(*msg, 15, "pitch " + signed_number(pitch_deg, 1) + " deg", point(p.x, p.y, p.z + 0.22), c));
    array.markers.push_back(text_marker(*msg, 16, "yaw " + signed_number(yaw_deg, 1) + " deg", point(p.x, p.y, p.z + 0.32), c));

    marker_pub_->publish(array);
  }

  static std::string signed_number(double value, int precision)
  {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%+.*f", precision, value);
    return std::string(buffer);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  std::string marker_namespace_;
  double axis_line_width_{0.01};
  double text_size_{0.08};
  double vehicle_arrow_length_{0.22};
  double vehicle_arrow_width_{0.05};
  double good_cov_trace_{0.02};
  double warn_cov_trace_{0.12};
};

}  // namespace relative_localization_eskf

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<relative_localization_eskf::PoseRvizMarkerNode>());
  rclcpp::shutdown();
  return 0;
}
