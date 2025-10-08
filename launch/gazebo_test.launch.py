from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import xacro
import os
import math

def generate_launch_description():
    x = 0.25
    y = 0.25
    z = 0.30
    theata = math.pi / 2

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    package_dir = get_package_share_directory("yasarobo2025_26")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    world = os.path.join(
        get_package_share_directory("yasarobo2025_26"), "worlds", "field.world"
    )

    # load robot urdf file
    xacro_file = os.path.join(package_dir, "urdf", "robot.xacro")
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        emulate_tty=True,
        parameters=[
            params,
            {"use_sim_time": use_sim_time}
        ]
    ) 

    # gazebo settings
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )


    return LaunchDescription([
        SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
        node_robot_state_publisher,
        gzserver_cmd,
        gzclient_cmd,
    ])