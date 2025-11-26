from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    rov_launch = FindPackageShare('rov_launch')

    default_config_path = PathJoinSubstitution([rov_launch, 'config', 'params.yaml'])
    ld.add_action(DeclareLaunchArgument(name="config",
                                        default_value=default_config_path,
                                        description="File name of configuration"))

    ld.add_action(Node(
        package='ds4_driver',
        executable='ds4_driver_node.py',
        output="screen"
    ))

    ld.add_action(Node(
        package="rov_passthrough_control",
        executable='base_node',
        parameters=[LaunchConfiguration('config')],
        output="screen"
    ))
    ld.add_action(Node(
        package='rov_thruster_manager',
        executable="thruster_manager",
        parameters=[LaunchConfiguration('config')],
        output="screen"
    ))
    ld.add_action(Node(
        package='rov_bridge',
        executable="stm32",
        parameters=[LaunchConfiguration('config')],
        output="screen"
    ))
    ld.add_action(Node(
        package='rov_description',
        executable='rov_state_publisher',
        parameters=[LaunchConfiguration('config')]
    ))
    return ld
