#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time
from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Inspection route, each array contains a position x/y and orientation z/w.
    # The robot will move to/orient at each waypoint in correspondance with where it believes it is.
    # Common 90 degree -orientations- are: 0.0,1.0 :: -1.0,0.0 :: -0.705,0.705 :: 0.705,0.705(maybe)
    inspection_route = [
        [0.0, 0.0, 0.0, 1.0],    #ORIENT FACING RAMP :: ACTIVATE INTAKE
        [0.35, 0.0, 0.0, 1.0],   #DRIVE FORWARDS
        [0.35, 0.0, -1.0, 0.0],   #ORIENT BACKWARDS
        [0.0, 0.0, -1.0, 0.0],    #DRIVE FORWARDS
        [0.0, 0.0, -0.705, 0.705],   #ORIENT BACKWARDS
        [0.0, -0.15, -0.705, 0.705],    #DRIVE FORWARDS :: CMD_VEL BACK UP     (change second value to get closer to wall)
        [0.0, 0.0, 0.0, 1.0],    #ORIENT FACING RAMP
        [0.3, 0.0, 0.0, 1.0],   #DRIVE FORWARDS
        [0.3, 0.0, -0.705, 0.705],   #ORIENT FACING BLOCKS
        [0.3, -0.15, -0.705, 0.705],   #DRIVE FORWARDS :: CMD_VEL BACK UP    (change second value to get closer to wall)
        [0.3, 0.0, 0.0, 1.0]]    #ORIENT FACING RAMPS :: ACTIVATE HAUL

    # Set our demo's initial pose
    # initial_pose = PoseStamped()
    # initial_pose.header.frame_id = 'map'
    # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    # initial_pose.pose.position.x = 3.45
    # initial_pose.pose.position.y = 2.15
    # initial_pose.pose.orientation.z = 1.0
    # initial_pose.pose.orientation.w = 0.0
    # navigator.setInitialPose(initial_pose)
    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()

    while rclpy.ok():

        # Send our route
        inspection_points = []
        inspection_pose = PoseStamped()
        inspection_pose.header.frame_id = 'map'
        inspection_pose.header.stamp = navigator.get_clock().now().to_msg() 
        for pt in inspection_route:
            inspection_pose.pose.position.x = pt[0]
            inspection_pose.pose.position.y = pt[1]
            inspection_pose.pose.orientation.z = pt[2]
            inspection_pose.pose.orientation.w = pt[3]
            inspection_points.append(deepcopy(inspection_pose))
        nav_start = navigator.get_clock().now()
        navigator.followWaypoints(inspection_points)

        # This comminicates with the waypoint_plugins and actually activates tasks at the waypoints
        i = 0
        while not navigator.isTaskComplete():
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Executing current waypoint: ' +
                    str(feedback.current_waypoint + 1) + '/' + str(len(inspection_points)))

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Waypoint Tasks Complete! Returning to Start...')
        elif result == TaskResult.CANCELED:
            print('Waypoint Tasks Canceled! Returning to Start...')
            exit(1)
        elif result == TaskResult.FAILED:
            print('Waypoint Tasks Failed! Returning to Start...')

        # go back to start
        # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        # navigator.goToPose(initial_pose)
        while not navigator.isTaskComplete:
            pass


if __name__ == '__main__':
    main()