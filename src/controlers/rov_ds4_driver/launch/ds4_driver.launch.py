from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    rov_ds4_driver = FindPackageShare('rov_ds4_driver')
    

    default_config_path = PathJoinSubstitution([rov_ds4_driver, 'config', 'params.yaml'])
    ld.add_action(DeclareLaunchArgument(name="config", default_value=default_config_path, description="File name of configuration"))

    ld.add_action(Node(
        package='ds4_driver',
        executable='ds4_driver_node.py',
        output="screen",
        parameters=[LaunchConfiguration('config')]
    ))

    ld.add_action(Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        parameters=[LaunchConfiguration('config')]
    ))

    return ld

