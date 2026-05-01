from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

from ament_index_python import get_package_share_directory


def generate_launch_description():

    kappelmeister = FindPackageShare('kappelmeister')

    default_config_path = PathJoinSubstitution([kappelmeister, 'config', 'bluerov2_pool.yaml'])
    default_rviz_config_path = PathJoinSubstitution([kappelmeister, 'rviz', 'bluerov2_pool.rviz'])

    # Launch arguments
    config_arg = DeclareLaunchArgument(
        name="config",
        default_value=default_config_path,
        description="Path to the configuration file for the stonefish simulator"
    )
    rviz_config_arg = DeclareLaunchArgument(
        name="rviz_config",
        default_value=default_rviz_config_path,
        description="Path to the RViz2 configuration file"
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen'
    )

    launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(get_package_share_directory('stonefish_ros2') + \
                                      '/launch/stonefish_simulator.launch.py'),
        launch_arguments={
            'simulation_data': get_package_share_directory('rov_stonefish') + '/data/',
            'scenario_desc': get_package_share_directory('rov_stonefish') + '/scenarios/tank.scn',
            'simulation_rate': '30.0',
            'window_res_x': '1820',
            'window_res_y': '980',
            'rendering_quality': 'low',
        }.items()
    )

    return LaunchDescription([
        config_arg,
        rviz_config_arg,
        rviz_node,
        launch_include,
    ])
