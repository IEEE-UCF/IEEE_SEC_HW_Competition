#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include <thread>

int current_time = 0;
int repeat_count = 0;

// CHANGE THIS TO RESTART UPON FAILURE AN EXACT NUMBER OF TIMES
const int repeat_tolerance = 4;

//CHANGE THIS IF YOU WANT ALL PROCESSES TO END UPON SUCCESS
const bool break_for_testing = false;




void current_launch_async(){
  
  //CHANGE THIS DEPENDING ON WHICH LAUNCH FILE YOU WANT - use_sim_time:=true may be needed
  std::system("ros2 launch secbot_bring_up combine_launch.py &");

}


void timerCallback(){
  
  //INCREASE MAX_WAIT IF THE FILE TAKES LONGER TO START
  const int max_wait = 15;
  current_time++;

  if((current_time % max_wait) == 0){
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "STARTUP FAILED - TRYING AGAIN");
    
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // CHANGE THIS DEPENDING ON NODES- ADD MORE NODES FOR HW
    std::system("pkill -2 -f 'bno055'");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    std::system("pkill -2 -f 'rplidar_composition'");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    std::system("pkill -2 -f 'robot_state_publisher'");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    std::system("pkill -2 -f 'gzserver'");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    std::system("pkill -2 -f 'ekf_node'");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    if(repeat_count < repeat_tolerance){
    repeat_count++;
    std::this_thread::sleep_for(std::chrono::seconds(3));
    current_time = 0;
    current_launch_async();
    }
    else{
      std::this_thread::sleep_for(std::chrono::seconds(6));
      rclcpp::shutdown();
    }
  }

}

auto checker_callback(const rcl_interfaces::msg::Log msg){



  //CHANGE ARRAY BASED ON MESSAGES YOU WISH TO SEE
  const char* gud_msg[] = {"Calling service /spawn_entity","Loaded gazebo_ros2_control.","\033[92mConfigured and activated \033[1mjoint_state_broadcaster\033[0m","\033[92mConfigured and activated \033[1mdiff_drive_controller\033[0m"};
  const char* lidar_msg[] = {"current scan mode: Sensitivity"};
  static int gud_msg_count = 0;

  if (strcmp(msg.msg.c_str(), gud_msg[gud_msg_count % 4]) == 0){
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "GOOD MESSAGE FOUND");
    
    gud_msg_count++;

  }

  if(msg.msg.size() > 25 && msg.msg.size() < 250 && msg.msg.at(4) == lidar_msg[0][4]){
  
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "FOUND LIDAR MESSAGE FOUND LIDAR MESSAGE");

    gud_msg_count++;

  }
  
  if(gud_msg_count == static_cast<int>(5)){
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "STARTUP WAS SUCCESFUL");

    if(break_for_testing == true){
      
      //CHANGE THIS DEPENDING ON NODES - HW WILL HAVE EXTRA HW NODES
      std::system("pkill -2 -f 'bno055'");
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      std::system("pkill -2 -f 'rplidar_composition'");
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      std::system("pkill -2 -f 'robot_state_publisher'");
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      std::system("pkill -2 -f 'gzserver'");
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      std::system("pkill -2 -f 'ekf_node'");
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DESTROYING LAUNCH SINCE TESTING ENABLED");
    }
    else{
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ENDING PROCESS WITH NODES ACTIVE");
    }
    //IF THERE ARE MANY NODES TO KILL AND TESTING ENABLED - INCREASE TIME
    std::this_thread::sleep_for(std::chrono::seconds(6));
    rclcpp::shutdown();

  }

}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("node_checker");

  auto checknode = node->create_subscription<rcl_interfaces::msg::Log>(
    "/rosout", rclcpp::SystemDefaultsQoS(), &checker_callback);
  
  auto timer = node->create_wall_timer(std::chrono::seconds(1), timerCallback);
  (void)std::async(std::launch::async, current_launch_async);

  rclcpp::spin(node);

  return 0;
}
