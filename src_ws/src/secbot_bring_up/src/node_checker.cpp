#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/log.hpp"


auto checker_callback(const rcl_interfaces::msg::Log msg){
  
  const char* gud_msg[] = {"Calling service /spawn_entity","Loaded gazebo_ros2_control."};
  static int gud_msg_count = 0;

  if (strcmp(msg.msg.c_str(), gud_msg[gud_msg_count]) == 0){
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "It worked");
    
    gud_msg_count++;

  }
  
  if(gud_msg_count == static_cast<int>(sizeof(gud_msg)/sizeof(gud_msg[0]))){
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Full Startup Succesful");

    rclcpp::shutdown();

  }

}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("node_checker");

  auto checknode = node->create_subscription<rcl_interfaces::msg::Log>(
    "/rosout", rclcpp::SystemDefaultsQoS(), &checker_callback);


  rclcpp::spin(node);

  return 0;
}
