#!/bin/bash

set -e


sudo apt-get update
#apt-get install -y apt-utils
#sudo apt-get install -y software-properties-common
#sudo add-apt-repository universe 
#sudo add-apt-repository multiverse

source /opt/ros/humble/install/setup.bash

pip3 install smbus
pip install pyserial

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update


#sudo apt install -y \
#  ros-$ROS_DISTRO-navigation2 \
#  ros-$ROS_DISTRO-nav2-bringup \


rosdep update --rosdistro=$ROS_DISTRO
sudo apt-get update
cd dependencies
rosdep install --from-paths src --ignore-src --skip-keys "gazebo_ros_pkgs" -r -y
. /opt/ros/${ROS_DISTRO}/setup.sh

sudo apt install -y ros-humble-rmw-cyclonedds-cpp




