# ball_tracker

This repository contains a simple demonstration of using ROS and OpenCV to track a ball with a mobile robot via a camera feed.

The associated tutorial is available [here](https://youtu.be/gISSSbYUZag).

Thanks very much to Tiziano Fiorenzani for his [ROS 1 tutorial](https://www.youtube.com/watch?v=We6CQHhhOFo) and [corresponding code](https://github.com/tizianofiorenzani/ros_tutorials/blob/master/opencv/src/find_ball.py) on which this repo is based (code reproduced with permission).

One other thing to be wary of: this is a ROS/ament Python package, which means the launch and config files contained will NOT be linked with the `--symlink-install` command for colcon, and will need to be rebuilt after changes.

## SEC Tips

The primary changes made by the SEC team were near the ends of the detect_ball_3d, detect_ball, and follow_ball nodes.

They each have a callback function that SHOULD listen to the primary cmd_vel topic. The follow_ball node will release smaller cmd vels as the robot appraoches the ball, so we made it to where the nodes would break once the cmd_vels became small enough numbers. Not only will the nodes automatically break, unlike before, but the node that generally dies last will send a process kill command to the launch file to ensure all associated ball tracker processes die.


## Getting started

A launch file that is referenced by the secbot_navigation package is included. Refer to the navigation package README to see how it's used.

## Known issues
- Can't easily detect red due to hue wrap. If you need to detect red robustly you'll need to go in and mess with the HSV filtering.
- Can only track and follow balls.
