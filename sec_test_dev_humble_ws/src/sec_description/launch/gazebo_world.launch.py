from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the directory of your ROS 2 package
    pkg_dir = get_package_share_directory('sec_description')
    world_file_path = os.path.join(get_package_share_directory(pkg_dir), 'worlds', 'comp_course.world')

    # Launch Gazebo with the specified world file
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file_path],
        output='screen'
    )

    return LaunchDescription([
        start_gazebo_cmd
    ])

