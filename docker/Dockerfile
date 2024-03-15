#FROM osrf/ros:humble-desktop-full as base
FROM dustynv/ros:humble-ros-base-l4t-r36.2.0 as base

# Example of installing programs
RUN apt-get update \
    && apt-get install -y \
    nano \
    vim \
    && rm -rf /var/lib/apt/lists/*


# Example of copying a file
#COPY config/ /site_config/


# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config


# Set up sudo
RUN apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*

RUN usermod -aG dialout ${USERNAME} 

#RUN apt-get update && mkdir -p ${ROS_ROOT}/src && cd ${ROS_ROOT}/src \
#    && git config --global http.postBuffer 524288000 \
#    && git clone https://github.com/ros-planning/navigation2.git && cd navigation2 && git checkout ec49c2772a0926c86ca83a4933c664744712e2e9 && cd .. \
#    && git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git && cd BehaviorTree.CPP && git checkout a363bdcae88350bc748598a7d2950e300859469c && cd .. \
#    && source ${ROS_ROOT}/install/setup.bash && cd ${ROS_ROOT} \
##    && git config http.postBuffer 524288000 \
#    && rosdep install -y -r --ignore-src --from-paths src --rosdistro ${ROS_DISTRO} --skip-keys "gazebo_ros_pkgs" \
#    && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --packages-up-to-regex nav2* --packages-ignore nav2_system_tests \
#    && rm -Rf src build log \
##&& rm -rf /var/lib/apt/lists/* \
##&& apt-get clean


# Copy the entrypoint and bashrc scripts so we have 
# our container's environment set up correctly
COPY docker/entrypoint.sh /entrypoint.sh

COPY docker/bashrc /home/${USERNAME}/.bashrc
COPY docker/downloadDependencies.sh /downloadDependencies.sh

COPY docker/bashrc /root/tmp/tmpbashrc
RUN cat /root/tmp/tmpbashrc >> /root/.bashrc  && rm -f /root/tmp/tmpbashrc

#copy 

# might need to copy the script over first
COPY src_ws /dependencies
run /downloadDependencies.sh


# Set up entrypoint and default command
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]
