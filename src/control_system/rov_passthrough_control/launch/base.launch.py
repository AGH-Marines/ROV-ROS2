from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    rov_passthrough_control = FindPackageShare('rov_passthrough_control')

    default_config_path = PathJoinSubstitution([rov_passthrough_control, 'config', 'params.yaml'])
    ld.add_action(DeclareLaunchArgument(name="config",
                                        default_value=default_config_path,
                                        description="File name of configuration"))

    ld.add_action(Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id': 0,        # /dev/input/js0
            'deadzone': 0.05,
            'autorepeat_rate': 20.0
        }]
    ))
    ld.add_action(Node(
        package="rov_passthrough_control",
        executable='base_node',
        parameters=[LaunchConfiguration('config')],
        output="screen"
    ))

    return ld
