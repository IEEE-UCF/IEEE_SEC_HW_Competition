#include "waypoint_action_follower/plugins/task_at_waypoint.hpp"

#include <string>
#include <memory>
#include <thread>

#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"

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
  subsystem_pub_ = node_->create_publisher<std_msgs::msg::String>("SubsystemComms",10);


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
    
    int SuccessValue;
    std_msgs::msg::String message;

    // PUBLISH VALUES FOR SUBSYSTEMS: 4 intake : 5 outake : 6 disable

    switch (curr_waypoint_index){
      case 0:          
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "AT FIRST WAYPOINT: Initiating Intake");

          //SEND MESSAGE TO TURN ON INTAKE
          message.data = "4"; 
          subsystem_pub_->publish(message);
          std::this_thread::sleep_for(std::chrono::seconds(4));

          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PROCESS COMPLETE");
          break;
      case 5:
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "AT SIXTH WAYPOINT: Initiating Backup..");

          //CHANGE -t VALUE TO BACK UP MORE OR LESS
          SuccessValue = std::system("ros2 topic pub -r 5 -t 7 /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist '{linear: {x: -0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'");
          //BREAKS IF LAUNCH CONTINUES SOMEWHOW
          if(SuccessValue != 0){throw std::runtime_error("MOVE STILL GOING SOMEHOW"); break;}
          
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PROCESS COMPLETE");
          break;
      case 9:
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "AT NINTH WAYPOINT: Initiating Backup..");

          //CHANGE -t VALUE TO BACK UP MORE OR LESS
          SuccessValue = std::system("ros2 topic pub -r 5 -t 7 /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist '{linear: {x: -0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'");
          //BREAKS IF LAUNCH CONTINUES SOMEWHOW
          if(SuccessValue != 0){throw std::runtime_error("MOVE STILL GOING SOMEHOW"); break;}
          
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PROCESS COMPLETE");
          break;
      case 10:
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "AT TENTH WAYPOINT: Disabling Intake and Initiating Haul Forwards..");

          
          //SEND MESSAGE TO TURN OFF INTAKE
          message.data = "5"; 
          subsystem_pub_->publish(message);
          std::this_thread::sleep_for(std::chrono::seconds(4));
          
          
          //CHANGE ANGULAR Z TO DETERMINE ADJUSTMENT VALUE BEFORE PASSING RAMP
          SuccessValue = std::system("ros2 topic pub -r 5 -t 3 /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}'");
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