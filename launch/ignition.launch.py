import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro
import math


def generate_launch_description():
    x = 0.25
    y = 0.25
    z = 0.30
    theata = math.pi/2

    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    package_dir = os.path.join(get_package_share_directory("yasarobo2025_26"))

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
        launch_arguments=[('gz_args', [f' -r 4 {world}'])]
    )

    xacro_file = os.path.join(package_dir, "urdf", "robot.xacro")
    doc = xacro.process_file(xacro_file, mappings={'use_sim' : 'true'})
    robot_desc = doc.toprettyxml(indent='  ')
    params = {'robot_description': robot_desc}

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
                   '-Y', str(theata)
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
            '/tf_static@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'],
        output='screen'
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
    )

    # nav2 map_server
    map_server_cmd = Node(
        package="nav2_map_server",
        executable="map_server",
        output="screen",
        parameters=[{'yaml_filename': map_server_config_path}]
    )

    # tf transfromer
    start_lifecycle_manager_cmd = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        output="screen",
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': lifecycle_nodes}]
    )

    static_from_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    mcl_node = Node(
        package="yasarobo2025_26",
        executable="mcl_node",
        parameters=[{
            "initial_x": x,
            "initial_y": y,
            "initial_theta": theata,
            "use_sim_time": use_sim_time
        }],
        output="screen"
    )

    # joy
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
        executable="vel_feedback_node",
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
            "use_sim_time": use_sim_time
        }],
    )

    follow_node = Node(
        package="yasarobo2025_26",
        executable="follow_node",
        output="screen",
        parameters=[{
            "max_linear_speed": 0.10,
            "max_angular_speed": 0.7,
            "lookahead_distance": 0.20,
            "resampleThreshold": 0.10,
            "Kp_linear": 1.0,
            "Ki_linear": 0.01,
            "Kd_linear": 0.0,
        }]
    )

    bt_node = Node (
        package="yasarobo2025_26",
        executable="bt_node",
        output="screen"
    )

    rotate_node = Node(
        package="yasarobo2025_26",
        executable="rotate_node",
        output="screen",
    )

    vacume_node = Node(
        package="yasarobo2025_26",
        executable="dummy_vacume_uart",
        output="screen"
    )

    detect_node = Node(
        package="yasarobo2025_26",
        executable="ball_detect_node",
        output="screen",
        parameters=[{
            "min_pts": 10,
            "wall_threshold": -1.0,
        }]
    )


    return LaunchDescription([
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
        detect_node
    ])
