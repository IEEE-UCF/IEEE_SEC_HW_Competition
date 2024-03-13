#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include <thread>

void launch_gazebo_async(){

  std::system("ros2 launch secbot_simulation launch_sim.launch.py &");
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Launching Gazebo Launch Now.");

}


auto checker_callback(const rcl_interfaces::msg::Log msg){

  const bool break_for_testing = true;

  static int times_called = 0;
  times_called++;

  const char* gud_msg[] = {"HAHAHAHHACalling service /spawn_entity","Loaded gazebo_ros2_control."};
  static int gud_msg_count = 0;

  if (strcmp(msg.msg.c_str(), gud_msg[gud_msg_count]) == 0){
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "It worked");
    
    gud_msg_count++;

  }
  
  if(gud_msg_count == static_cast<int>(sizeof(gud_msg)/sizeof(gud_msg[0]))){
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Full Startup Succesful");

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "times called: %d", times_called);

    if(break_for_testing == true){
      
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Destroying nodes since testing enabled");
      std::system("pkill -2 -f 'robot_state_publisher'");
      std::system("pkill -2 -f 'gzserver'");

    }

    rclcpp::shutdown();

  }

  
  if(times_called==18 && gud_msg_count != static_cast<int>(sizeof(gud_msg)/sizeof(gud_msg[0]))){
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Startup Failed, Rerunning Gazebo");
      std::system("pkill -2 -f 'robot_state_publisher'");
      std::system("pkill -2 -f 'gzserver'");

      std::this_thread::sleep_for(std::chrono::milliseconds(300));

      launch_gazebo_async();

  }

  

}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("node_checker");

  auto checknode = node->create_subscription<rcl_interfaces::msg::Log>(
    "/rosout", rclcpp::SystemDefaultsQoS(), &checker_callback);

  (void)std::async(std::launch::async, launch_gazebo_async);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "STARTING CHECKER NODE");

  rclcpp::spin(node);

  return 0;
}
