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
    print(map_server_config_path)
    lifecycle_nodes = ['map_server']

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
        ),
        launch_arguments={
            'gzclient_args': '--render-fps 15',
            'extra_gzclient_env': 'LIBGL_ALWAYS_SOFTWARE=1'
        }.items()
    )

    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                    '-entity', 'robot',
                    '-x', str(x),
                    '-y', str(y),
                    '-z', str(z),
                    '-Y', str(theata),
                ],
        output='screen',
        emulate_tty=True,
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

    # nav2 map_server
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
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': lifecycle_nodes}]
    )

    # tf transfromer
    static_from_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=['0', '0', '-0.255', '0', '0', '0', 'map', 'odom']
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


    return LaunchDescription([
        SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=spawn_entity,
        #         on_exit=[load_joint_state_broadcaster],
        #     )
        # ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=load_joint_state_broadcaster,
        #         on_exit=[load_diff_drive_base_controller],
        #     )
        # ),
        node_robot_state_publisher,
        gzserver_cmd,
        gzclient_cmd,
        spawn_entity,
        rviz_node,
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
        vacume_node
    ])