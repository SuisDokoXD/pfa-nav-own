import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('gimbal_bridge')
    default_params = os.path.join(share, 'config', 'gimbal_bridge.yaml')

    params_file = LaunchConfiguration('params_file')
    port = LaunchConfiguration('port')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0',
                              description='serial device of the C board'),

        Node(
            package='gimbal_bridge',
            executable='gimbal_bridge_node',
            name='gimbal_bridge',
            output='screen',
            parameters=[params_file, {'port': port}],
        ),
    ])
