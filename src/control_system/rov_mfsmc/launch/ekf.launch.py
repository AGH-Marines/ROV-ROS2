from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

from ament_index_python import get_package_share_directory

def generate_launch_description():

    mfsmc_share = FindPackageShare('rov_mfsmc')
    thruster_manager_share = FindPackageShare('rov_thruster_manager')
    description_share = FindPackageShare('rov_description')
    wrench_system_share = FindPackageShare('rov_wrench_system')

        # Define default configuration paths
    default_config_path = PathJoinSubstitution([mfsmc_share, 'config', 'params.yaml'])

    # Launch arguments
    config_arg = DeclareLaunchArgument(
        name="config",
        default_value=default_config_path,
        description="Path to the configuration file for the stonefish simulator"
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[LaunchConfiguration('config')],
        output='screen'
    )

    return LaunchDescription([
        config_arg,
        robot_localization_node
    ])