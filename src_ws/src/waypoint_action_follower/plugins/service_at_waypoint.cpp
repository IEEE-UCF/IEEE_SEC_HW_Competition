#include "rclcpp/rclcpp.hpp"
#include "waypoint_interfaces/srv/waypoint_index.hpp"
#include "waypoint_action_follower/plugins/service_at_waypoint.hpp"

#include <string>
#include <memory>

#include "pluginlib/class_list_macros.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_waypoint_follower
{

ServiceAtWaypoint::ServiceAtWaypoint()
: is_enabled_(true)
{
}

ServiceAtWaypoint::~ServiceAtWaypoint()
{
}

  

void ServiceAtWaypoint::initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
    const std::string &plugin_name)
{
  auto node_ = parent.lock();
  if (!node_)
  {
    throw std::runtime_error("Failed to lock parent node");
  }
  
  logger_ = node_->get_logger();
  
  std::string velocity_topic;

  nav2_util::declare_parameter_if_not_declared(
    node_, plugin_name + ".enabled",
    rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node_, plugin_name + ".velocity_topic",
    rclcpp::ParameterValue("/diff_drive_controller/cmd_vel_unstamped"));
  node_->get_parameter(plugin_name + ".enabled", is_enabled_);
  node_->get_parameter(plugin_name + ".velocity_topic", velocity_topic);


  cmd_vel_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>(
        velocity_topic, 1);
}

bool ServiceAtWaypoint::processAtWaypoint(
    const geometry_msgs::msg::PoseStamped & /*curr_pose*/,
    const int & curr_waypoint_index)
{
  if (!is_enabled_) {
    return true;
  }    
  try {
    // Create a Twist message to send desired velocities
    geometry_msgs::msg::Twist twist_cmd;
    twist_cmd.angular.z = 100.0;  // Set the desired angular velocity, adjust as needed

    // Publish the Twist message
    cmd_vel_publisher_->publish(twist_cmd);

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
    nav2_waypoint_follower::ServiceAtWaypoint,
    nav2_core::WaypointTaskExecutor)