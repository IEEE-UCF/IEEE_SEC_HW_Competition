#include <memory>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
using std::placeholders::_1;

#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

class TFSubscriber : public rclcpp::Node
{
public:
  TFSubscriber() : Node("tf_subscriber")
  {
    subscription_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", 10, std::bind(&TFSubscriber::tf_callback, this, std::placeholders::_1));
  }

private:
  void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    for (const auto& transform : msg->transforms) {
      RCLCPP_INFO(
          this->get_logger(),
          "Received TF message:\nFrame ID: %s\nChild Frame ID: %s\nRotation: (%f, %f, %f, %f)",
          transform.header.frame_id.c_str(),
          transform.child_frame_id.c_str(),
          transform.transform.rotation.x,
          transform.transform.rotation.y,
          transform.transform.rotation.z,
          transform.transform.rotation.w);

          std::this_thread::sleep_for(std::chrono::milliseconds(125));
    }
  }

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TFSubscriber>());
  rclcpp::shutdown();
  return 0;
}
