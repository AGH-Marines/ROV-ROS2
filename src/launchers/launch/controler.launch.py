from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

from ament_index_python import get_package_share_directory


def generate_launch_description():

    launchers_share = FindPackageShare('launchers')

    # Define default configuration paths
    default_config_path = PathJoinSubstitution([launchers_share, 'config', 'wreckage_bluerov2.yaml'])
    config_arg = DeclareLaunchArgument(
        name="config",
        default_value=default_config_path,
        description="Path to the configuration file for the stonefish simulator"
    )
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id': 0,  # /dev/input/js0
            'deadzone': 0.05,
            'autorepeat_rate': 20.0
        }]
    )

    return LaunchDescription([
        config_arg,
        joy_node
    ])
