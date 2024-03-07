#include "rclcpp/rclcpp.hpp"
#include "waypoint_interfaces/srv/waypoint_index.hpp"                                        

#include<thread>
#include<chrono>
#include <memory>
#include <cstdlib>

void add(const std::shared_ptr<waypoint_interfaces::srv::WaypointIndex::Request> request,     
          std::shared_ptr<waypoint_interfaces::srv::WaypointIndex::Response>       response)  
{
  response->recieved = 0;                                               
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Recived Waypoint Index");

  switch ((request->index)+1)
  {
      case 1:
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "WAYPOINT 1: Initiating the ball tracker for 15 seconds..");
          std::system("ros2 launch secbot_navigation ball_tracker_launch.py &");
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ball tracker started, sleeping for 15 seconds..");
          std::this_thread::sleep_for(std::chrono::seconds(15));
          std::system("pkill -f ball_tracker_launch.py");
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "System has killed the ball_tracker launch..");
          std::this_thread::sleep_for(std::chrono::seconds(2));
          break;
      case 2:
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "WAYPOINT 2: Initiating the ball tracker for 15 seconds..");
          std::system("ros2 launch secbot_navigation ball_tracker_launch.py &");
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ball tracker started, sleeping for 15 seconds..");
          std::this_thread::sleep_for(std::chrono::seconds(15));
          std::system("pkill -f ball_tracker_launch.py");
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "System has killed the ball_tracker launch..");
          std::this_thread::sleep_for(std::chrono::seconds(2));
          break;
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending success message back to waypoint_follower. Onto the next waypoint..: [%ld]", (long int)response->recieved);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("waypoint_index_server");   

  rclcpp::Service<waypoint_interfaces::srv::WaypointIndex>::SharedPtr service =               
    node->create_service<waypoint_interfaces::srv::WaypointIndex>("waypoint_index",  &add);   

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to recieve the index");                     

  rclcpp::spin(node);
  rclcpp::shutdown();
}