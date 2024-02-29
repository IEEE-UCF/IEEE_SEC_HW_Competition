#include "serial/serial.h"

#include <iostream>
#include "thread"
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

int movebih();

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::String & msg) const
    {
      //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
      movebih();
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}







int movebih()
{
    serial::Serial my_serial("/dev/ttyUSB0", 19200, serial::Timeout::simpleTimeout(3000));

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