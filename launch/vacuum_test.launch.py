import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # 1. 起動引数の定義（デフォルトでは実機用のノードを起動、Trueにするとダミーを起動）
    use_dummy_arg = DeclareLaunchArgument(
        'use_dummy',
        default_value='false',
        description='Use dummy_vacume_uart if true, otherwise use real vacume_uart.'
    )

    use_dummy = LaunchConfiguration('use_dummy')

    # 2. 実機用真空ノード (vacume_uart)
    # /dev/serial/by-path/... を開くため、PC/Raspberry Piにデバイスが接続されている必要があります
    vacume_node = Node(
        package="yasarobo2025_26",
        executable="vacume_uart",
        output="screen",
        condition=UnlessCondition(use_dummy)
    )

    # 3. ダミー真空ノード (dummy_vacume_uart)
    # シリアルデバイスを必要とせず、固定のカラー（1: YELLOW）を返します
    dummy_vacume_node = Node(
        package="yasarobo2025_26",
        executable="dummy_vacume_uart",
        output="screen",
        condition=IfCondition(use_dummy)
    )

    return LaunchDescription([
        # ログ出力をカラー表示にする設定
        SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
        
        use_dummy_arg,
        vacume_node,
        dummy_vacume_node
    ])