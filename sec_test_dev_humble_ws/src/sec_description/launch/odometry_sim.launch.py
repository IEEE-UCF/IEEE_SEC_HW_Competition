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

    # Specify the name of the package and path to xacro file within the package
    package_name ='sec_description'

    ekf_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'ekf.yaml')
    rviz_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'config.rviz')

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
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    start_spawn_entity = Node(package='gazebo_ros', 
                        executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'sec_bot'],
                        output='screen')

    start_rviz = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [rviz_params_file]]
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
    ld.add_action(start_spawn_entity)
    ld.add_action(start_rviz)
    ld.add_action(start_robot_localization)

    return ld