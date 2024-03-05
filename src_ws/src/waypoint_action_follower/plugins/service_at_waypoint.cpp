#include "rclcpp/rclcpp.hpp"
#include "waypoint_interfaces/srv/waypoint_index.hpp"
#include "waypoint_action_follower/plugins/service_at_waypoint.hpp"

#include <chrono>
#include <cstdlib>
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
  

  nav2_util::declare_parameter_if_not_declared(
    node_, plugin_name + ".enabled",
    rclcpp::ParameterValue(true));
  node_->get_parameter(plugin_name + ".enabled", is_enabled_);

}

bool ServiceAtWaypoint::processAtWaypoint(
    const geometry_msgs::msg::PoseStamped & /*curr_pose*/,
    const int & curr_waypoint_index)
{
  if (!is_enabled_) {
    return true;
  }    
  try {
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("waypoint_index_client");  
  rclcpp::Client<waypoint_interfaces::srv::WaypointIndex>::SharedPtr client =                
    node->create_client<waypoint_interfaces::srv::WaypointIndex>("waypoint_index");          

  auto request = std::make_shared<waypoint_interfaces::srv::WaypointIndex::Request>();       
  request->index = curr_waypoint_index;                                     // THIS NEED TO BE REPLACED WITH WAYPOINT INDEX                         

  while (!client->wait_for_service(std::chrono::milliseconds(1000))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "recieved: %ld", result.get()->recieved);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service waypoint_index");    
  }
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