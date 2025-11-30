import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch_ros

import xacro
import math
import random


def generate_launch_description():
    x = 0.25
    y = 0.25
    z = 0.30
    theta = math.pi/2
    frequency = 25.0


    # cpu simulation setting
    os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'

    package_dir = get_package_share_directory("yasarobo2025_26")

    world = os.path.join(
        get_package_share_directory("yasarobo2025_26"), "worlds", "field.world"
    )
    map_server_config_path = os.path.join(
        package_dir,
        "map",
        "map.yaml"
    )
    rviz_config_path = os.path.join(
        package_dir,
        "config",
        "default.rviz"
    )
    lifecycle_nodes = ['map_server']

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
        launch_arguments=[('gz_args', [f' -r -s {world}'])]
    )

    xacro_file = os.path.join(package_dir, "urdf", "robot.xacro")
    doc = xacro.process_file(xacro_file, mappings={'use_sim' : 'true'})
    robot_desc = doc.toprettyxml(indent='  ')
    params = {'robot_description': robot_desc}

    # load ball urdf file
    ball_urdf_file = os.path.join(package_dir, "urdf", "ball.urdf")

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_desc,
                   '-name', 'robot',
                   '-allow_renaming', 'false',
                   '-x', str(x),
                   '-y', str(y),
                   '-z', str(z),
                   '-Y', str(theta)
                ],
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/ldlidar_node/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/tf_static@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/world/yasarobo/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'],
        output='screen'
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
        remappings=[('clock', '/world/yasarobo/clock')]
    )

    # nav2 map_server
    map_server_cmd = Node(
        package="nav2_map_server",
        executable="map_server",
        output="screen",
        parameters=[
            {'yaml_filename': map_server_config_path},
        ],
        remappings=[('clock', '/world/yasarobo/clock')]
    )

    # tf transfromer
    start_lifecycle_manager_cmd = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        output="screen",
        emulate_tty=True,
        parameters=[
            {'autostart': True},
            {'node_names': lifecycle_nodes}],
        remappings=[('clock', '/world/yasarobo/clock')]
    )

    static_from_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        remappings=[('clock', '/world/yasarobo/clock')]
    )

    mcl_node = Node(
        package="yasarobo2025_26",
        executable="mcl_node",
        parameters=[
            {
                "initial_x": x,
                "initial_y": y,
                "initial_theta": theta,
                "scanStep": 5,
            },
        ],
        remappings=[('clock', '/world/yasarobo/clock')],
        output="screen"
    )

    # joy
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        remappings=[('clock', '/world/yasarobo/clock')],
    )

    joy2Vel_node = Node(
        package="yasarobo2025_26",
        executable="joy2vel",
        name="joy2vel",
        output="screen",
        remappings=[('clock', '/world/yasarobo/clock')],
    )

    vel_feedback_node = Node(
        package="yasarobo2025_26",
        executable="vel_feedback_node",
        output="screen",
        remappings=[('clock', '/world/yasarobo/clock')],
    )

    gen_path = Node(
        package="yasarobo2025_26",
        executable="gen_path",
        output="screen",
        parameters=[
            {
                "initial_x": x,
                "initial_y": y,
                "initial_theta": theta,
                "sample_parameter": frequency,
            },
        ],
        remappings=[('clock', '/world/yasarobo/clock')],
    )

    follow_node = Node(
        package="yasarobo2025_26",
        executable="follow_node",
        output="screen",
        parameters=[{
            "max_linear_speed": 0.10,
            "max_angular_speed": 0.7,
            "max_linear_tolerance": 0.05,
            "max_theta_tolerance": 0.10,
            "max_reaching_distance": 0.05,
            "max_reaching_theta": 0.10,
            "lookahead_distance": 0.20,
            "resampleThreshold": 0.10,
            "Kp_tan": 0.80,
            "Ki_tan": 0.0,
            "Kd_tan": 0.0,
            "Kp_normal": 0.80,
            "Ki_normal": 0.00,
            "Kd_normal": 0.00,
            "Kp_theta": 1.0,
            "Ki_theta": 0.00,
            "Kd_theta": 0.00,
            "x": 10,
        },
        ],
        remappings=[('clock', '/world/yasarobo/clock')],
    )

    bt_node = Node (
        package="yasarobo2025_26",
        executable="bt_node",
        output="screen",
        remappings=[('clock', '/world/yasarobo/clock')],
    )

    rotate_node = Node(
        package="yasarobo2025_26",
        executable="rotate_node",
        output="screen",
        remappings=[('clock', '/world/yasarobo/clock')],
    )

    vacume_node = Node(
        package="yasarobo2025_26",
        executable="dummy_vacume_uart",
        output="screen",
        remappings=[('clock', '/world/yasarobo/clock')],
    )

    detect_node = Node(
        package="yasarobo2025_26",
        executable="ball_detect_node",
        output="screen",
        parameters=[{
            "min_pts": 10,
            "wall_threshold": -1.0,
        },
        ],
        remappings=[('clock', '/world/yasarobo/clock')],
    )

    ball_path_node = Node(
        package="yasarobo2025_26",
        executable="ball_path_node",
        output="screen",
        remappings=[('clock', '/world/yasarobo/clock')],
        parameters=[{
            "shorten": 0.15,
            "num_points_": 10
        }]
    )

    # spawn ball on field
    ball_spawn_entity_list = []
    ball_x_min = 0.98
    ball_x_max = 1.70
    ball_y_min = 0.60
    ball_y_max = 1.80
    for i_x in range(2):
        for i_y in range(0,1,1):
            region_x_min = ball_x_min + (ball_x_max-ball_x_min)*i_x/2
            region_x_max = ball_x_min + (ball_x_max-ball_x_min)*(i_x+1)/2
            region_y_min = ball_y_min + (ball_y_max-ball_y_min)*i_y/4
            region_y_max = ball_y_min + (ball_y_max-ball_y_min)*(i_y+1)/4

            for _ in range(1):
                ball_x = random.uniform(region_x_min, region_x_max)
                ball_y = random.uniform(region_y_min, region_y_max)
                ball_spawn_entity_list.append(
                    Node(
                        package='ros_gz_sim',
                        executable='create',
                        output='screen',
                        arguments=[
                            '-file', str(ball_urdf_file),
                            '-name', 'ball',
                            '-x', str(ball_x),
                            '-y', str(ball_y),
                            '-z', str(z),
                            '-allow_renaming', 'true'
                        ],
                    )
                )

    return LaunchDescription([
        launch_ros.actions.SetParameter(name='use_sim_time', value=True),
        gazebo,
        node_robot_state_publisher,
        gz_spawn_entity,
        bridge,
        rviz,
        map_server_cmd,
        start_lifecycle_manager_cmd,
        static_from_map_to_odom,
        mcl_node,
        joy_node,
        joy2Vel_node,
        vel_feedback_node,
        gen_path,
        follow_node,
        rotate_node,
        bt_node,
        vacume_node,
        detect_node,
        ball_path_node,
        *ball_spawn_entity_list
    ])
