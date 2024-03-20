#include "waypoint_action_follower/plugins/task_at_waypoint.hpp"

#include <string>
#include <memory>
#include <thread>

#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_waypoint_follower
{

TaskAtWaypoint::TaskAtWaypoint()
: is_enabled_(true)
{
}

TaskAtWaypoint::~TaskAtWaypoint()
{
}
  

void TaskAtWaypoint::initialize(
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

bool TaskAtWaypoint::processAtWaypoint(
    const geometry_msgs::msg::PoseStamped & /*curr_pose*/,
    const int & curr_waypoint_index)
{
  if (!is_enabled_) {
    return true;
  }    

  try {
    //START THE WAYPOINT PROCCESS DEPENDING ON THE WAYPOINT INDEX    
    switch (curr_waypoint_index){
      case 0:          
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "AT FIRST WAYPOINT: Initiating Drive Over..");

          //STARTS BALL TRACKER - SET i EQUAL TO THE NUMBER OF CUBES OR BALLS
          for(int i=0;i<1;i++){
            int SuccessValue = std::system("ros2 launch secbot_navigation ball_tracker_launch.py");
          
            //BREAKS IF LAUNCH CONTINUES SOMEWHOW
            if(SuccessValue != 0){throw std::runtime_error("LAUNCH STILL GOING SOMEHOW"); break;}
          
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "BALL TRACKER FINISHED - GRABBING BALL");
            std::this_thread::sleep_for(std::chrono::seconds(3));
          }

          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PROCESS COMPLETE");
          break;
      case 1:
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "AT SECOND WAYPOINT: Initiating Ball Tracker..");

          //CHANGE ANGULAR Z TO DETERMINE ADJUSTMENT VALUE BEFORE PASSING RAMP
          int SuccessValue = std::system("ros2 topic pub -r 5 -t 3 /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}'");
          //BREAKS IF LAUNCH CONTINUES SOMEWHOW
          if(SuccessValue != 0){throw std::runtime_error("MOVE STILL GOING SOMEHOW"); break;}

          SuccessValue = std::system("ros2 topic pub -r 5 -t 15 /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist '{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'");
          //BREAKS IF LAUNCH CONTINUES SOMEWHOW
          if(SuccessValue != 0){throw std::runtime_error("MOVE STILL GOING SOMEHOW"); break;}
          
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PROCESS COMPLETE");
          break;

    }

    RCLCPP_INFO(
      rclcpp::get_logger("nav2_waypoint_follower"),
      "Completed task at waypoint %i, returning to navigation..", curr_waypoint_index);

  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("nav2_waypoint_follower"),
      "Failed to start task at waypoint %i! Caught exception: %s",
      curr_waypoint_index, e.what());
    return false;
  }
  return true;
}
}  // namespace nav2_waypoint_follower

PLUGINLIB_EXPORT_CLASS(
    nav2_waypoint_follower::TaskAtWaypoint,
    nav2_core::WaypointTaskExecutor)