from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from ament_index_python import get_package_share_directory


def shutdown_on_exit(node, reason):
    return RegisterEventHandler(
        OnProcessExit(
            target_action=node,
            on_exit=[Shutdown(reason=reason)]
        )
    )

def generate_launch_description():
    mfsmc_share = FindPackageShare('rov_mfsmc')
    autonomy_system_share = FindPackageShare('autonomy_system')
    passthrough_share = FindPackageShare('rov_passthrough_control')

    default_config_path = PathJoinSubstitution([mfsmc_share, 'config', 'params.yaml'])
    default_autonomy_config_path = PathJoinSubstitution([autonomy_system_share, 'config', 'params.yaml'])
    default_rviz_config_path = PathJoinSubstitution([mfsmc_share, 'rviz', 'tf_basic.rviz'])

    # Define default configuration paths
    default_config_path = PathJoinSubstitution([mfsmc_share, 'config', 'params.yaml'])

    # Launch arguments
    config_arg = DeclareLaunchArgument(
        name="config",
        default_value=default_config_path,
        description="Path to the configuration file for the stonefish simulator"
    )
    autonomy_config_arg = DeclareLaunchArgument(
        name="autonomy_config",
        default_value=default_autonomy_config_path,
        description="Path to the autonomy configuration file"
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

    passthrough_launch = IncludeLaunchDescription(
        launch_description_source=PathJoinSubstitution([passthrough_share, 'launch', 'base.launch.py']),
        launch_arguments={
            "config": LaunchConfiguration('config')
        }.items()
    )

    # traj_gen_node = Node(
    #     package='traj_gen',
    #     executable='min_snap_traj_generator',
    #     parameters=[LaunchConfiguration('config')],
    #     output='screen'
    # )

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
        executable='rov_mfsmc_better_node',
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

    stabilize_on_position_action = Node(
        package='autonomy_system',
        executable='stabilize_on_position_action',
        name='stabilize_on_position_action_node',
        parameters=[LaunchConfiguration('autonomy_config')],
        output='screen',
        emulate_tty=True
    )

    stabilize_on_position_2_action = Node(
        package='autonomy_system',
        executable='stabilize_on_position_action',
        name='stabilize_on_position_action_2_node',
        parameters=[LaunchConfiguration('autonomy_config')],
        output='screen',
        emulate_tty=True
    )

    behavior_tree = Node(
        package='autonomy_system',
        executable='behavior_tree',
        name='behavior_tree',
        parameters=[LaunchConfiguration('autonomy_config')],
        output='screen',
        emulate_tty=True
    )

    description_timer = TimerAction(period=1.0, actions=[rov_state_publisher_node])
    rviz_timer = TimerAction(period=1.0, actions=[rviz_node])
    stonefish_timer = TimerAction(period=2.0, actions=[launch_include])
    traj_gen_timer = TimerAction(period=3.0, actions=[tf_traj_gen])
    tasks_timer = TimerAction(period=3.0, actions=[stabilize_on_position_action,stabilize_on_position_2_action, behavior_tree])

    return LaunchDescription([
        config_arg,
        rviz_config_arg,
        autonomy_config_arg,
        robot_localization_node,
        thruster_manager_node,
        passthrough_launch,
        mfsm_node,
        description_timer,
        rviz_timer,
        stonefish_timer,
        traj_gen_timer,
        tasks_timer
    ])
