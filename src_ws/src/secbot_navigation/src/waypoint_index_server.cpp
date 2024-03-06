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

  std::future<int> process_future;

  switch ((request->index)+1)
  {
      case 1:
          process_future = std::async(std::launch::async, [](){
              return std::system("ros2 launch secbot_navigation ball_tracker_launch.py");
          });
          std::this_thread::sleep_for(std::chrono::seconds(10));

          std::system("pkill -f ball_tracker_launch.py");

          // system("ros2 launch secbot_navigation ball_tracker_launch.py");
          // std::this_thread::sleep_for(std::chrono::seconds(10));
          // rclcpp::shutdown();
          break;
      case 2:
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "At second waypoint, sleeping for 10 seconds");
          std::this_thread::sleep_for(std::chrono::seconds(10));
          break;
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending Back Response: [%ld]", (long int)response->recieved);
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