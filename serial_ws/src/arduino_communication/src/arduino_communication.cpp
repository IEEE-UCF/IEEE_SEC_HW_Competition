#include "serial/serial.h"

#include <chrono>
#include <functional>
#include <string>

#include <iostream>
#include "thread"
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using namespace std::chrono_literals;
using std::placeholders::_1;

int movebih();
void processEncoder();

class SerialCom : public rclcpp::Node
{
  public:
    SerialCom()
    : Node("motor_com")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&SerialCom::topic_callback, this, _1));

      encoder1_ = this->create_publisher<std_msgs::msg::String>("Encoder1", 10);
      timer = this->create_wall_timer(
        500ms, std::bind(&MotorCom::processEncoder, this));
    }

  private:
    void topic_callback(const std_msgs::msg::String & msg) const
    {
      //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
      movebih();
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr encoder1_;
    rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialCom>());
  rclcpp::shutdown();
  return 0;
}

// function for sending a command through serial
int movebih()
{
    serial::Serial my_serial("/dev/ttyUSB0", 115200, serial::Timeout::simpleTimeout(3000));

    if (my_serial.isOpen())
    {
        std::cout << "Port opened succesfully" << std::endl;
    }
    else
    {
        std::cout << "Port failed to open" << std::endl;
    }
    my_serial.flushOutput();

    std::string test_string = "move_forward(100000)";
    for (int i = 0; i < 3; i++)
    {
        size_t bytesWritten = my_serial.write(test_string);

        // std::string result = my_serial.read(test_string.length() + 1);
        std::cout << "Bytes sent: " << bytesWritten << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
    test_string = "move_backward(100000)";
    for (int i = 0; i < 3; i++)
    {
        size_t bytesWritten = my_serial.write(test_string);

        // std::string result = my_serial.read(test_string.length() + 1);
        std::cout << "Bytes sent: " << bytesWritten << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    my_serial.flushInput();
    std::string response = my_serial.read(6);
    std::cout << "Arduino: " << response << std::endl;
}

  // function for reading encoder value and displaying it to a topic
  void processEncoder() {
    serial::Serial my_serial("/dev/ttyUSB0", 115200, serial::Timeout::simpleTimeout(3000));
    if (my_serial.isOpen())
    {
        //std::cout << "Port opened succesfully" << std::endl;
    }
    else
    {
        //std::cout << "Port failed to open" << std::endl;
        return;
    }
    my_serial.flushOutput();
    // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    my_serial.flushInput();
    std::string response = my_serial.read(16);
    if (!response.empty()) {
      auto message = std_msgs::msg::String();
      message.data = response;
        my_serial.flushInput();
        my_serial.flushOutput();
        RCLCPP_INFO(this->get_logger(), "Encoder: '%s'", message.data.c_str());
        encoder1_->publish(message);
      }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  
  // function for reading a command from a topic and sending through serial
  void readCommand (const std_msgs::msg::String & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    serial::Serial my_serial("/dev/ttyUSB7", 115200, serial::Timeout::simpleTimeout(3000));
    if (my_serial.isOpen())
    {
        //std::cout << "Port opened succesfully" << std::endl;
    }
    else
    {
        //std::cout << "Port failed to open" << std::endl;
        return;
    }
    
    my_serial.flushOutput();
    my_serial.flushInput();
  //std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    my_serial.write(msg.data.c_str());
  }