from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # 1. vel_feedback_uart ノードの起動
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

    # 2. /cmd_vel へ一定の指令値をパブリッシュするコマンド
    # linear.x を 0.1 m/s で動かし続ける例
    # 必要に応じて --rate を変更してください
    pub_cmd_vel = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--rate', '20',
            '/cmd_vel', 'geometry_msgs/msg/Twist',
            '{linear: {x: 0.0, y: 0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
        ],
        output='screen'
    )

    return LaunchDescription([
        vel_feedback_node,
        pub_cmd_vel
    ])