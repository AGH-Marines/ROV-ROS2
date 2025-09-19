from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    description_share = FindPackageShare('rov_description')

    default_config_path = PathJoinSubstitution([description_share, 'config', 'params.yaml'])

    config_arg = DeclareLaunchArgument(name="config",
                                       default_value=default_config_path,
                                       description="Path to the configuration file for the rov state publisher")

    rov_state_publisher_node = Node(package='rov_description',
                                    executable='rov_state_publisher',
                                    parameters=[LaunchConfiguration('config')])

    return LaunchDescription([config_arg,
                              rov_state_publisher_node])
