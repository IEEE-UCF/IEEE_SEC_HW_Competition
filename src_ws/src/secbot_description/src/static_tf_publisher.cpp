#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("static_tf_publisher");

    auto clock_now = rosgraph_msgs::msg::Clock();

    // Create a subscriber for the /clock topic
    auto clock_subscriber = node->create_subscription<rosgraph_msgs::msg::Clock>(
        "/clock", rclcpp::SystemDefaultsQoS(), [&clock_now](const rosgraph_msgs::msg::Clock::SharedPtr msg) {
            clock_now = *msg;
        });

    // Create a TF broadcaster
    tf2_ros::TransformBroadcaster tf_broadcaster(node);

    // Set the common transform values
    geometry_msgs::msg::TransformStamped lidar_transform;
    lidar_transform.header.frame_id = "chassis";  // Change "world" to your reference frame
    lidar_transform.child_frame_id = "lidar_link";
    lidar_transform.transform.translation.x = 0.0;  // Set your desired translation values
    lidar_transform.transform.translation.y = 0.0;
    lidar_transform.transform.translation.z = 0.0;
    lidar_transform.transform.rotation.x = 0.0;  // Set your desired rotation values
    lidar_transform.transform.rotation.y = 0.0;
    lidar_transform.transform.rotation.z = 0.0;
    lidar_transform.transform.rotation.w = 1.0;

    // Main loop
    rclcpp::Rate loop_rate(20);  // Adjust the loop rate as needed

    while (rclcpp::ok()) {

        lidar_transform.header.stamp.sec = clock_now.clock.sec+0.2;
        lidar_transform.header.stamp.nanosec = clock_now.clock.nanosec+200000000;


        tf_broadcaster.sendTransform(lidar_transform);

        // Spin once to process callbacks
        rclcpp::spin_some(node);

        // Sleep to maintain the loop rate
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
