from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
import launch.logging

import xacro
import os
import math

def generate_launch_description():
    package_dir = get_package_share_directory("yasarobo2025_26")
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value="bag",
        description='Path to the ROS2 bag to play'
    )
    bag_path = LaunchConfiguration('bag_path')

    ros_bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', PathJoinSubstitution([
             TextSubstitution(text=os.path.expanduser("~/ros_bags")),
             bag_path
         ])],
        output='screen'
    )

    rviz_config_path = os.path.join(
        package_dir,
        "config",
        "default.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        emulate_tty=True,
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["-d", rviz_config_path]
    )

    return LaunchDescription([
        bag_path_arg,
        ros_bag,
        rviz_node
    ])