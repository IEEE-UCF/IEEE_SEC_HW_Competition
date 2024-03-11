#ifndef NAV2_WAYPOINT_FOLLOWER_PLUGINS_SERVICE_AT_WAYPOINT_HPP_
#define NAV2_WAYPOINT_FOLLOWER_PLUGINS_SERVICE_AT_WAYPOINT_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_core/waypoint_task_executor.hpp"

namespace nav2_waypoint_follower
{

/**
 * @brief Plugin based on WaypointTaskExecutor, turns the robot at waypoint arrival.
 */
class ServiceAtWaypoint : public nav2_core::WaypointTaskExecutor
{
public:
  /**
   * @brief Construct a new Turn Robot At Waypoint object
   */
  ServiceAtWaypoint();

  /**
   * @brief Destructor
   */
  ~ServiceAtWaypoint();

  /**
   * @brief Initializes the plugin
   *
   * @param parent The parent node
   * @param plugin_name The name of the plugin
   */
  void initialize(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
      const std::string &plugin_name);

  /**
   * @brief Process the action of turning the robot at the waypoint
   *
   * @param curr_pose The current pose of the robot
   * @param curr_waypoint_index The index of the waypoint
   * @return true if the action is successful, false otherwise
   */
  bool processAtWaypoint(
      const geometry_msgs::msg::PoseStamped & curr_pose,
      const int & curr_waypoint_index);

protected:
  /**
   * @brief Processor callback
   * @param msg Empty message
   */
  bool is_enabled_;
  rclcpp::Logger logger_{rclcpp::get_logger("waypoint_action_follower")};
};

} // namespace nav2_waypoint_follower

#endif // NAV2_WAYPOINT_FOLLOWER_PLUGINS_SERVICE_AT_WAYPOINT_HPP_
