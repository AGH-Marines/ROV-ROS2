from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

from ament_index_python import get_package_share_directory

def generate_launch_description():

    controll_bus_share = FindPackageShare('rov_controll_bus')

    # Define default configuration paths
    default_config_path = PathJoinSubstitution([controll_bus_share, 'config', 'params.yaml'])

    # Launch arguments
    config_arg = DeclareLaunchArgument(
        name="config",
        default_value=default_config_path,
        description="Path to the configuration file for the stonefish simulator"
    )

    controll_bus_node = Node(
        package='rov_controll_bus',
        executable='controll_bus',
        name='n_controller_bus',
        parameters=[LaunchConfiguration('config')],
        output='screen'
    )

    return LaunchDescription([
        config_arg,
        controll_bus_node
    ])