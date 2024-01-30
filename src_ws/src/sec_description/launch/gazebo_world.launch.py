from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_name = 'sec_description'
    world_file_name = 'comp_course.world'

    # Get the directory of your ROS 2 package
    pkg_dir = get_package_share_directory(pkg_name)
    world_file_path = os.path.join(pkg_dir, 'worlds', world_file_name)

    gazebo_model_path = os.path.join(get_package_share_directory(pkg_name), 'models')
    set_model_path = SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path)

    # Launch Gazebo with the specified world file
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file_path],
        output='screen'
    )

    return LaunchDescription([
        set_model_path,
        start_gazebo_cmd
    ])