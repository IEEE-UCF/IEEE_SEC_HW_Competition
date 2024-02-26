
#include "turn_robot_at_waypoint.hpp"

#include <string>
#include <memory>

#include "pluginlib/class_list_macros.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_waypoint_follower
{
TurnRobotAtWaypoint::TurnRobotAtWaypoint()
{
}

TurnRobotAtWaypoint::~TurnRobotAtWaypoint()
{
}

void TurnRobotAtWaypoint::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name)
{
  auto node = parent.lock();

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".enabled",
    rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".velocity_topic",
    rclcpp::ParameterValue("/diff_drive_controller/cmd_vel_unstamped"));

  std::string velocity_topic;
  node->get_parameter(plugin_name + ".enabled", is_enabled_);
  node->get_parameter(plugin_name + ".velocity_topic", velocity_topic);

  // Create a publisher to send velocity commands
  velocity_publisher_ = node->create_publisher<geometry_msgs::msg::Twist>(
    velocity_topic, rclcpp::QoS(10));

  if (!is_enabled_) {
    RCLCPP_INFO(
      rclcpp::get_logger("nav2_waypoint_follower"),
      "TurnRobotAtWaypoint plugin is disabled.");
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger("nav2_waypoint_follower"),
      "Initializing TurnRobotAtWaypoint plugin, publishing velocities to topic: %s",
      velocity_topic.c_str());
  }
}

bool TurnRobotAtWaypoint::processAtWaypoint(
  const geometry_msgs::msg::PoseStamped & curr_pose, const int & curr_waypoint_index)
{
  if (!is_enabled_) {
    RCLCPP_WARN(
      rclcpp::get_logger("nav2_waypoint_follower"),
      "TurnRobotAtWaypoint plugin is disabled. Not performing anything"
    );
    return true;
  }

  try {
    // Create a Twist message to send desired velocities
    geometry_msgs::msg::Twist twist_cmd;
    twist_cmd.angular.z = 1.0;  // Set the desired angular velocity, adjust as needed

    // Publish the Twist message
    velocity_publisher_->publish(twist_cmd);

    RCLCPP_INFO(
      rclcpp::get_logger("nav2_waypoint_follower"),
      "Turning the robot at waypoint %i", curr_waypoint_index);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("nav2_waypoint_follower"),
      "Failed to send velocity command at waypoint %i! Caught exception: %s",
      curr_waypoint_index, e.what());
    return false;
  }

  return true;
}

}  // namespace nav2_waypoint_follower

PLUGINLIB_EXPORT_CLASS(
  nav2_waypoint_follower::TurnRobotAtWaypoint,
  nav2_core::WaypointTaskExecutor)