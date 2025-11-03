from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

from ament_index_python import get_package_share_directory


def generate_launch_description():

    position_snapshot = FindPackageShare('rov_position_snapshot')
    wrench_system_share = FindPackageShare('rov_passthrough_control')

    default_config_path = PathJoinSubstitution([position_snapshot, 'config', 'params.yaml'])
    default_rviz_config_path = PathJoinSubstitution([position_snapshot, 'rviz', 'tf_basic.rviz'])

    # Define default configuration paths
    default_config_path = PathJoinSubstitution([position_snapshot, 'config', 'params.yaml'])

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

    thruster_manager_node = Node(
        package='rov_thruster_manager',
        executable="thruster_manager",
        parameters=[LaunchConfiguration('config')],
        output="screen"
    )

    rov_state_publisher_node = Node(
        package='rov_description',
        executable='rov_state_publisher',
        parameters=[LaunchConfiguration('config')]
    )

    wrench_system_launch = IncludeLaunchDescription(
        launch_description_source=PathJoinSubstitution([wrench_system_share, 'launch', 'base.launch.py']),
        launch_arguments={
            "config": LaunchConfiguration('config')
        }.items()
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[LaunchConfiguration('config')],
        output='screen'
    )

    position_snapshot = Node(
        package='rov_position_snapshot',
        executable='rov_position_snapshot_node',
        arguments=[LaunchConfiguration('config')],
        output='screen'
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

    description_timer = TimerAction(period=1.0, actions=[rov_state_publisher_node])
    rviz_timer = TimerAction(period=1.0, actions=[rviz_node])
    stonefish_timer = TimerAction(period=2.0, actions=[launch_include])

    return LaunchDescription([
        config_arg,
        rviz_config_arg,
        robot_localization_node,
        description_timer,
        rviz_timer,
        stonefish_timer,
        thruster_manager_node,
        wrench_system_launch,
        position_snapshot
    ])
