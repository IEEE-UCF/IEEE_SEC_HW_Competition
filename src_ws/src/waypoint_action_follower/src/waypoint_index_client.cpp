#include "rclcpp/rclcpp.hpp"
#include "waypoint_interfaces/srv/waypoint_index.hpp"                                       

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // if (argc != 4) { 
  //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: waypoint_index_client X Y Z");      
  //     return 1;
  // }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("waypoint_index_client");  
  rclcpp::Client<waypoint_interfaces::srv::WaypointIndex>::SharedPtr client =                
    node->create_client<waypoint_interfaces::srv::WaypointIndex>("waypoint_index");          

  auto request = std::make_shared<waypoint_interfaces::srv::WaypointIndex::Request>();       
  request->index = 4;                                     // THIS NEED TO BE REPLACED WITH WAYPOINT INDEX                         

  while (!client->wait_for_service(1s)) {
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

  rclcpp::shutdown();
  return 0;
}