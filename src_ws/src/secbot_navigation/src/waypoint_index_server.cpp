#include "rclcpp/rclcpp.hpp"
#include "waypoint_interfaces/srv/waypoint_index.hpp"                                        

#include <memory>

void add(const std::shared_ptr<waypoint_interfaces::srv::WaypointIndex::Request> request,     
          std::shared_ptr<waypoint_interfaces::srv::WaypointIndex::Response>       response)  
{
  response->recieved = request->index;                                      
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\nindex: %ld",  
                request->index);                                         
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->recieved);
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