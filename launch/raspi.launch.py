from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import os
import xacro
import math

def generate_launch_description():
    x = 0.25
    y = 0.25
    z = 0.30
    theata = math.pi / 2

    package_dir = get_package_share_directory("yasarobo2025_26")

    # load robot urdf file
    xacro_file = os.path.join(package_dir, "urdf", "robot.xacro")
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    map_server_config_path = os.path.join(
        package_dir,
        "map",
        "map.yaml"
    )
    lifecycle_nodes = ['map_server']

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        emulate_tty=True,
        parameters=[
            params,
        ]
    ) 

    map_server_cmd = Node(
        package="nav2_map_server",
        executable="map_server",
        output="screen",
        parameters=[{'yaml_filename': map_server_config_path}]
    )
    start_lifecycle_manager_cmd = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        output="screen",
        emulate_tty=True,
        parameters=[
            {'autostart': True},
            {'node_names': lifecycle_nodes}
        ]
    )

    # #ldlidar
    ldlidar_params = PathJoinSubstitution(
        [FindPackageShare("yasarobo2025_26"), "config", "ldlidar_settings.yaml"]
    )

    ldlidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ldlidar_node"), "launch", "ldlidar_with_mgr.launch.py"]
            )
        ),
        launch_arguments={"params_file": ldlidar_params}.items(),
    )

    # tf transfromer
    static_from_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=['0', '0', '-0.255', '0', '0', '0', 'map', 'odom']
    )

    static_ldlidar_tf = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='ldlidar_tf',
    arguments=[
        # 3D 位置 (x  y  z)
        '0.0', '0.0', '0.00',
        # 回転 (roll pitch yaw) でも quaternion(x y z w) でも可
         '1.570', '0', '0',
        # 親フレーム / 子フレーム
        'ldlidar_base', 'ldlidar_link'
    ],
)

#     static_from_odom_to_basefootprint = Node(
#     package="tf2_ros",
#     executable="static_transform_publisher",
#     name="static_odom_to_basefootprint",
#     output="screen",
#     arguments=[
#         "0.25",          # x  [m]
#         "0.25",          # y  [m]
#         "0.30",             # z  [m]
#         "0",             # yaw   [rad]
#         "0",             # pitch [rad]
#         "0",             # roll  [rad]
#         "odom",          # parent  frame
#         "base_footprint" # child   frame
#     ]
# )

    mcl_node = Node(
        package="yasarobo2025_26",
        executable="mcl_node",
        output="screen",
        parameters=[{
            "initial_x": x,
            "initial_y": y,
            "initial_theta": theata,
            "odomNoise1": 4.0,
            "odomNoise2": 3.0,
            "odomNoise3": 4.0,
            "odomNoise4": 3.0,
        }],
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
    )

    joy2Vel_node = Node(
        package="yasarobo2025_26",
        executable="joy2vel",
        name="joy2vel",
        output="screen"
    )

    vel_feedback_node = Node(
        package="yasarobo2025_26",
        executable="vel_feedback_uart",
        output="screen"
    )

    vacume_node = Node(
        package="yasarobo2025_26",
        executable="vacume_uart",
        output="screen"
    )

    gen_path = Node(
        package="yasarobo2025_26",
        executable="gen_path",
        output="screen",
        parameters=[{
            "initial_x": x,
            "initial_y": y,
            "initial_theta": theata,
        }],
    )

    follow_node = Node(
        package="yasarobo2025_26",
        executable="follow_node",
        output="screen",
        parameters=[{
            "max_linear_speed": 0.10,
            "max_angular_speed": 0.5,
            "lookahead_distance": 0.10,
        }]
    )

    rotate_node = Node(
        package="yasarobo2025_26",
        executable="rotate_node",
        output="screen",
    )

    bt_node = Node (
        package="yasarobo2025_26",
        executable="bt_node",
        output="screen"
    )

    # cmd_velをキャッチして、uartに流すプログラムが必要
    return LaunchDescription([
        node_robot_state_publisher,
        map_server_cmd,
        start_lifecycle_manager_cmd,
        static_from_map_to_odom,
        mcl_node,
        joy_node,
        joy2Vel_node,
        vel_feedback_node,
        ldlidar_node,
        static_ldlidar_tf,
        gen_path,
        follow_node,
        vacume_node,
        bt_node,
        rotate_node,
        # static_from_odom_to_basefootprint
    ])
