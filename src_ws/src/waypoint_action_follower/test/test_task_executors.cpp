// Copyright (c) 2021, Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.

#include <math.h>
#include <condition_variable>
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "waypoint_action_follower/plugins/wait_at_waypoint.hpp"
#include "waypoint_action_follower/plugins/turn_robot_at_waypoint.hpp"


class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

TEST(WaypointFollowerTest, WaitAtWaypoint)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testWaypointNode");

  node->declare_parameter("WAW.waypoint_pause_duration", 50);

  std::unique_ptr<nav2_waypoint_follower::WaitAtWaypoint> waw(
    new nav2_waypoint_follower::WaitAtWaypoint
  );
  waw->initialize(node, std::string("WAW"));

  auto start_time = node->now();

  // should wait 50ms
  geometry_msgs::msg::PoseStamped pose;
  waw->processAtWaypoint(pose, 0);

  auto end_time = node->now();

  EXPECT_NEAR((end_time - start_time).seconds(), 0.05, 0.01);

  waw.reset(new nav2_waypoint_follower::WaitAtWaypoint);
  node->set_parameter(rclcpp::Parameter("WAW.enabled", false));
  waw->initialize(node, std::string("WAW"));

  // plugin is not enabled, should exit
  EXPECT_TRUE(waw->processAtWaypoint(pose, 0));
}

TEST(WaypointFollowerTest, TurnRobotAtWaypoint)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testWaypointNode");
  auto pub = node->create_publisher<geometry_msgs::msg::Twist>("/diff_drive_controller/cmd_vel_unstamped", 1);
  pub->on_activate();

  auto publish_message = [&]() -> void
    {
      rclcpp::Rate(5).sleep();
      auto msg = std::make_unique<geometry_msgs::msg::Twist>();
      // fill twist msg data.
      msg->angular.z = 1.0;
      pub->publish(std::move(msg));
      rclcpp::spin_some(node->shared_from_this()->get_node_base_interface());
    };

  // Create and initialize the TurnRobotAtWaypoint instance
  std::unique_ptr<nav2_waypoint_follower::TurnRobotAtWaypoint> traw(
    new nav2_waypoint_follower::TurnRobotAtWaypoint
  );
  traw->initialize(node, std::string("TRAW"));

  // Call processAtWaypoint with no twist published, should return false
  geometry_msgs::msg::PoseStamped pose;
  EXPECT_FALSE(traw->processAtWaypoint(pose, 0));

  // Publish a twist message in a separate thread
  std::thread t1(publish_message);

  // Now, there is a twist available, processAtWaypoint should return true
  EXPECT_TRUE(traw->processAtWaypoint(pose, 0));
  t1.join();


  // Reset the TurnRobotAtWaypoint instance and disable it
  traw.reset(new nav2_waypoint_follower::TurnRobotAtWaypoint);
  node->set_parameter(rclcpp::Parameter("TRAW.enabled", false));
  traw->initialize(node, std::string("TRAW"));

  // The plugin is not enabled, should exit and return true
  EXPECT_TRUE(traw->processAtWaypoint(pose, 0));
}