#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include <thread>

int current_time = 0;
int worry_msg_count = 0;

//HOW LONG THE SYSTEM WILL WAIT FOR ERRORS - EVEN IF IT FINDS ALL GOOD MESSAGES
//SHOULD BE LOWER THAN MAX_WAIT_BREAK
const int max_wait_success = 200;

const int max_wait_break = 8;

//CHANGE THIS IF YOU WANT ALL PROCESSES TO END UPON SUCCESS
const bool break_for_testing = true;

void end_all_nodes(){
      // CHANGE THIS DEPENDING ON NODES
      std::system("pkill -2 -f 'ball_tracker/follow_ball'");
      std::this_thread::sleep_for(std::chrono::seconds(2));
      std::system("pkill -2 -f 'ball_tracker/detect_ball_3d'");
      std::this_thread::sleep_for(std::chrono::seconds(2));
      std::system("pkill -2 -f 'ball_tracker/detect_ball'");
      std::this_thread::sleep_for(std::chrono::seconds(2));
      std::system("pkill -2 -f 'secbot_navigation/server'");
      std::this_thread::sleep_for(std::chrono::seconds(2));
      std::system("pkill -2 -f 'follow_waypoints.py'");
      std::this_thread::sleep_for(std::chrono::seconds(2));
      std::system("pkill -9 -f 'follow_waypoints.py'");
      std::this_thread::sleep_for(std::chrono::seconds(2));
      std::system("pkill -2 -f 'waypoint_launch.py'");
      std::this_thread::sleep_for(std::chrono::seconds(2));         
      std::system("pkill -2 -f 'component_container_isolated'");
      std::this_thread::sleep_for(std::chrono::seconds(4));         
      std::system("pkill -9 -f 'component_container_isolated'");
      std::this_thread::sleep_for(std::chrono::seconds(5)); 
      std::system("pkill -15 -f 'bringup_launch.py'");
      std::this_thread::sleep_for(std::chrono::seconds(4));  
      std::system("pkill -2 -f 'ekf_node'");
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));      
      std::system("pkill -2 -f 'gzserver'");
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));      
      std::system("pkill -2 -f 'robot_state_publisher'");
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}


void current_launch_async(){

  //CHANGE THIS DEPENDING ON WHICH LAUNCH FILE YOU WANT - HW WILL HAVE EXTRA NODES
  std::system("ros2 launch secbot_navigation hw_waypoint_launch.py&");

}

void succesful_startup(){
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ENDED NODE SINCE REACHED DESIRED TIME");

    if(break_for_testing == true){
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DESTROYING SUCCESSFULL LAUNCH SINCE TESTING ENABLED");
      std::this_thread::sleep_for(std::chrono::seconds(2));      
      
      end_all_nodes();
 
    }
    else{
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ENDING CURRENT PROCESS WITH ROBOT ACTIVE");
    }
    //IF THERE ARE MANY NODES TO KILL AND TESTING ENABLED - INCREASE TIME
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "NODE SHUTDOWN IS ALMOST OVER");
    std::this_thread::sleep_for(std::chrono::seconds(6));
    rclcpp::shutdown();

}


void timerCallback(){
  
  current_time++;
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TIME: %d", current_time);
  
  //IF CHECK CONDITIONS ARE MET
  if(current_time > max_wait_success){
    succesful_startup();
  }
  if(current_time > max_wait_break && worry_msg_count > 4){
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "FAILED TO CONNECT TO AMCL - ENDING PROCCESSES");
    end_all_nodes();
    std::this_thread::sleep_for(std::chrono::seconds(6));
    rclcpp::shutdown();

  }

}

auto checker_callback(const rcl_interfaces::msg::Log msg){

  //CHANGE ARRAY BASED ON MESSAGES YOU WISH TO SEE
  const char* bad_msg[] = {"Message Filter dropping message: frame 'lidar_link' at time 75.246 for reason 'the timestamp on the message is earlier than all the data in the transform cache'"};
  const char* worry_msg[] = {"amcl/get_state service not available, waiting..."};
  

//  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SEARCHING");

  if (strcmp(msg.msg.c_str(), worry_msg[0]) == 0){
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "WORRY MESSAGE FOUND");
    
    worry_msg_count++;

  }

  //IF EITHER ERROR MSG IS FOUND
  if(msg.name == "amcl" && msg.msg.size() > 158 && msg.msg.size() < 164 && msg.msg.at(40) == bad_msg[0][40]){
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "FOUND BAD MESSAGE FOUND BAD MESSAGE");

    end_all_nodes();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "FOUND ERRORS - RESETTING PROCESS");
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