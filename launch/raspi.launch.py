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
from launch.event_handlers import OnProcessExit, OnProcessStart
import datetime

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

    # rosbag
    bag_dir = os.path.expanduser("/misc/usb/ros_bags")
    # os.makedirs(bag_dir, exist_ok=True)
    ros_bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '-o', os.path.join(bag_dir, datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))],
        output='screen'
    )

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
            "particleNum": 50,
            "initial_x": x,
            "initial_y": y,
            "initial_theta": theata,
            "odomNoise1": 2.0,
            "odomNoise2": 0.5,
            "odomNoise3": 2.0,
            "odomNoise4": 5.0,
            "resampleThreshold": 0.9,
            "scanStep": 5,
            "lidar_threshold": 3.0/40.0*math.pi,
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
        output="screen",
        parameters=[{
            "Kp_linear": 0.1,
            "Kp_angular": 0.05,
            "max_linear_acceleration": 0.5,
            "max_angular_acceleration": 10.0
        }]
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
            "max_linear_speed": 0.13,
            "max_angular_speed": 0.5,
            "lookahead_distance": 0.10,
            "max_linear_tolerance": 0.25,
            "max_reaching_distance": 0.04,
            "max_theta_tolerance": 1.00,
            "max_reaching_theta": 0.10,
            "Kp_tan": 0.80,
            "Ki_tan": 0.00,
            "Kd_tan": 0.00,
            "Kp_norm": 0.80,
            "Ki_norm": 0.00,
            "Kd_norm": 0.00,
            "Kp_theta": 1.00,
            "Ki_theta": 0.00,
            "Kd_theta": 0.00,
            "x":25
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

    ball_path_node = Node(
        package="yasarobo2025_26",
        executable="ball_path_node",
        output="screen",
        remappings=[('clock', '/world/yasarobo/clock')],
        parameters=[{
            "shorten": 0.10,
            "num_points_": 10,
            "theta_offset": math.pi/20.0,
        }]
    )


    detect_node = Node(
        package="yasarobo2025_26",
        executable="ball_detect_node",
        output="screen",
        parameters=[{
            "eps": 0.020,
            "min_pts": 5,
            "wall_threshold": 0.0,
            "diff_threshold": 1e-8,
            "lidar_threshold": 3.0/40.0*math.pi,
            "radius_threshold": 0.05
        }]
    )

    return LaunchDescription([
        SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
        ros_bag,
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=ros_bag,
                on_start=[
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
                    ball_path_node,
                    detect_node
                ]
            )
        )
    ])
