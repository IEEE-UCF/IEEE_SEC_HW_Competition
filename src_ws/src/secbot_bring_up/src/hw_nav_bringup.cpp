#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include <thread>

int current_time = 0;
int gud_msg_count = 0;
int times_failed = 0;

//CHANGE THIS IF YOU WANT NAV2 TO GIVE UP AFTER CERTAIN NUMBER OF FAILURES
const int reset_max = 1;

//INCREASE MAX_WAIT IF THE FILE TAKES LONGER TO START
const int max_wait_break = 25;

//HOW LONG THE SYSTEM WILL WAIT FOR ERRORS - EVEN IF IT FINDS ALL GOOD MESSAGES
//SHOULD BE LOWER THAN MAX_WAIT_BREAK
const int max_wait_success = 15;

//CHANGE THIS IF YOU WANT ALL PROCESSES TO END UPON SUCCESS
const bool break_for_testing = false;

//CHANGE THIS DEPENDING ON HOW MANY GOOD MSGS YOU PUT GUD_MSGS
int max_good_msgs = 2;



void end_all_nodes(){
      // CHANGE THIS DEPENDING ON NODES - ADD EXTRA PROCESSES TO KILL FOR HW
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

  //CHANGE THIS DEPENDING ON WHICH LAUNCH FILE YOU WANT
  std::system("ros2 launch secbot_navigation bringup_launch.py &");

}


void gazebo_reset(){

  times_failed++;

  if(times_failed < reset_max){

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  end_all_nodes();
  std::this_thread::sleep_for(std::chrono::seconds(6));

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ALL PROCESSES ENDED - RESTARTING GAZEBO NOW");
  
  std::this_thread::sleep_for(std::chrono::seconds(2));
  std::system("ros2 run secbot_bring_up hw_launch_bringup");

  current_time = 0;
  gud_msg_count = 0;
  current_launch_async();
    
  }
  else{
    end_all_nodes();
    
    std::this_thread::sleep_for(std::chrono::seconds(6));
    rclcpp::shutdown();
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "NAVIGATION FAILED TOO MANY TIMES - SHUT SYSTEM DOWN");
  }
}


void succesful_startup(){
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "STARTUP WAS SUCCESFUL");

    if(break_for_testing == true){
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DESTROYING SUCCESSFULL LAUNCH SINCE TESTING ENABLED");
      std::this_thread::sleep_for(std::chrono::seconds(2));      
      
      end_all_nodes();
 
    }
    else{
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ENDING PROCESS WITH NODES ACTIVE");
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "NODE SHUTDOWN IS ALMOST OVER");

    //IF THERE ARE MANY NODES TO KILL AND TESTING ENABLED - INCREASE TIME
    std::this_thread::sleep_for(std::chrono::seconds(6));
    rclcpp::shutdown();

}


void timerCallback(){
  
  current_time++;
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TIME: %d", current_time);
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MSGS: %d", gud_msg_count);


  
  //IF CHECK CONDITIONS ARE MET
  if(current_time > max_wait_success && gud_msg_count >= 1){
    succesful_startup();
  }
  //IF WANTED MSGS AND CONDITIONS ARENT MET
  else if(current_time > max_wait_break && gud_msg_count == 0){
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "NO GOOD MSGS FOUND - STARTUP FAILED - RESETING PROCESSES");
    gazebo_reset();
  }
}

auto checker_callback(const rcl_interfaces::msg::Log msg){

  //CHANGE ARRAY BASED ON MESSAGES YOU WISH TO SEE
  const char* gud_msg[] = {"\033[34m\033[1mCreating bond timer...\033[0m\033[0m","Timed out waiting for transform from base_link to odom to become available, tf error: Invalid frame ID \"odom\" passed to canTransform argument target_frame - frame does not exist",
  "Message Filter dropping message: frame 'lidar_link' at time 212.620 for reason 'discarding message because the queue is full'"};
  
  //IF STRING IN ARRAY IS FOUND
  if (strcmp(msg.msg.c_str(), gud_msg[0]) == 0){
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "GOOD MESSAGE FOUND");
    
    gud_msg_count++;

  }
  //IF EITHER ERROR MSG IS FOUND
  else if(strcmp(msg.msg.c_str(), gud_msg[1]) == 0 || 
   (msg.name == "amcl" && msg.msg.size() > 158 && msg.msg.size() < 164 )){
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "FOUND ERRORS - RESETTING PROCESSES");
    gazebo_reset();
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
