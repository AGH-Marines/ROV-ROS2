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
    wrench_system_share = FindPackageShare('rov_wrench_system')

    default_config_path = PathJoinSubstitution([mfsmc_share, 'config', 'params.yaml'])
    default_rviz_config_path = PathJoinSubstitution([mfsmc_share, 'rviz', 'tf_basic.rviz'])

    # Define default configuration paths
    default_config_path = PathJoinSubstitution([mfsmc_share, 'config', 'params.yaml'])

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

    traj_gen_node = Node(
        package='traj_gen',
        executable='min_snap_traj_generator',
        arguments=['-d', LaunchConfiguration('config')],
        output='screen'
    )

    tf_traj_gen = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["0", "0", "0", "0", "0", "0", "world_ned", "traj_gen"]
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[LaunchConfiguration('config')],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen'
    )

    mfsm_node = Node(
        package='rov_mfsmc',
        executable='rov_mfsmc_node',
        parameters=[LaunchConfiguration('config')],
        output='screen'
    )

    launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(get_package_share_directory('stonefish_ros2') + \
                                      '/launch/stonefish_simulator.launch.py'),
        launch_arguments={
            'simulation_data': get_package_share_directory('rov_stonefish') + '/data/',
            'scenario_desc': get_package_share_directory('rov_stonefish') + '/scenarios/windturbine_bluerov2.scn',
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
        tf_traj_gen,
        traj_gen_node,
        thruster_manager_node,
        wrench_system_launch,
        mfsm_node
    ])
