from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    package_dir = get_package_share_directory("yasarobo2025_26")

    rviz_config_path = os.path.join(
        package_dir,
        "config",
        "default.rviz"
    )

    # rviz2 settings
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        emulate_tty=True,
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["-d", rviz_config_path]
    )

    # visualizer
    plotjuggler_ndoe = Node(
        package="plotjuggler",
        executable="plotjuggler",
        name="plotjuggler",
        output="screen",
    )

    return LaunchDescription([
        SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
        rviz_node,
        plotjuggler_ndoe
    ])