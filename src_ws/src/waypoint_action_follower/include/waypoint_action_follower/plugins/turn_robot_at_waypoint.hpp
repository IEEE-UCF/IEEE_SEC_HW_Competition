#ifndef NAV2_WAYPOINT_FOLLOWER_PLUGINS_TURN_ROBOT_AT_WAYPOINT_HPP_
#define NAV2_WAYPOINT_FOLLOWER_PLUGINS_TURN_ROBOT_AT_WAYPOINT_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_core/waypoint_task_executor.hpp"

namespace nav2_waypoint_follower
{

class TurnRobotAtWaypoint : public nav2_core::WaypointTaskExecutor
{
public:
  TurnRobotAtWaypoint();

  ~TurnRobotAtWaypoint();

  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name);

  bool processAtWaypoint(
    const geometry_msgs::msg::PoseStamped & curr_pose, const int & curr_waypoint_index);

protected:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  bool is_enabled_;
  // Other parameters or member variables specific to turning the robot
};

}  // namespace nav2_waypoint_follower

#endif  // NAV2_WAYPOINT_FOLLOWER_PLUGINS_TURN_ROBOT_AT_WAYPOINT_HPP_
