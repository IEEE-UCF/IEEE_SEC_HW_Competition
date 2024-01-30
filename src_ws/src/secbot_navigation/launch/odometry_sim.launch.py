import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    """
    To view live odom -> base_link transform:
        ros2 run tf2_ros tf2_echo odom base_link

    To view live ekf_node info
        ros2 node info /ekf_filter_node

    For live topic info:
        ros2 topic info imu_broadcaster/imu
        ros2 topic info diff_drive_controller/odom
        p.s. add -v flag to see node names for pubs and subs        
    """

    description_package_name = 'secbot_description'
    simulation_package_name = 'secbot_simulation'
    navigation_package_name = 'secbot_navigation'

    ekf_params_file = os.path.join(get_package_share_directory(navigation_package_name), 'config', 'ekf.yaml')

    # Launch config variables specific to sim
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_robot_localization = LaunchConfiguration('use_robot_localization')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_use_robot_localization = DeclareLaunchArgument(
        name='use_robot_localization',
        default_value='true',
        description='Use robot_localization if true'
    )

    start_robot_state_publisher = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(description_package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    start_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    start_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', [os.path.join(get_package_share_directory(description_package_name), 'config', 'config.rviz')]]
    )


    start_robot_localization = Node(
        condition=IfCondition(use_robot_localization),
        package='robot_localization',
        executable='ekf_node',
        parameters=[ekf_params_file]
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_robot_localization)

    ld.add_action(start_robot_state_publisher)
    ld.add_action(start_robot_localization)
    ld.add_action(start_joint_state_publisher_gui)
    ld.add_action(start_rviz2)

    return ld