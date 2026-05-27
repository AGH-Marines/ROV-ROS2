from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Udziały pakietów
    launchers_share = FindPackageShare('launchers')
    pid_share = FindPackageShare('rov_pid')
    thruster_manager_share = FindPackageShare('rov_thruster_manager')

    # Domyślne ścieżki konfiguracji
    default_config_path = PathJoinSubstitution([pid_share, 'config', 'params.yaml'])
    default_rviz_config_path = PathJoinSubstitution([thruster_manager_share, 'rviz', 'thruster.rviz'])

    # Argumenty launch
    config_arg = DeclareLaunchArgument(
        name="config",
        default_value=default_config_path,
        description="Path to the configuration file"
    )

    rviz_config_arg = DeclareLaunchArgument(
        name="rviz_config",
        default_value=default_rviz_config_path,
        description="Path to the RViz2 configuration file for ROV and thrusters visualization"
    )

    # Węzeł EKF (robot_localization)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[LaunchConfiguration('config')],
        output='screen'
    )

    # Węzeł PID
    pid_node = Node(
        package='rov_pid',
        executable='rov_pid_node',
        parameters=[LaunchConfiguration('config')],
        output='screen'
    )

    # Węzeł Zarządzania Pędnikami
    thruster_manager_node = Node(
        package='rov_thruster_manager',
        executable="thruster_manager",
        parameters=[LaunchConfiguration('config')],
        output="screen"
    )

    # Węzeł mostka (rov_bridge) - komunikacja z mikrokontrolerem
    rov_bridge = Node(
        package="rov_bridge",
        executable='uard',
        parameters=[LaunchConfiguration('config')],
        output="screen"
    )

    # Węzeł konwersji Joystick -> Cel TF (dla stabilizacji)
    joy_to_target_tf_node = Node(
        package='rov_passthrough_control',
        executable='joy_to_target_tf',
        parameters=[{
            'parent_frame': 'base_link',
            'target_frame': 'traj_gen_node',
            'base_frame': 'base_link',
            'max_lin_vel': 0.3,
            'max_ang_vel': 0.3
        }],
        output='screen'
    )

    # Driver pada (zakładając rov_ds4_driver lub standardowy joy_node)
    # Jeśli użytkownik używa rov_ds4_driver, można go tu dodać.
    # Na razie zakładamy, że joy node jest uruchamiany oddzielnie lub dodajemy standardowy:
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'deadzone': 0.1}]
    )

    # Statyczne transformacje dla ZED 2i i IMU (jeśli potrzebne)
    tf_zed = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.2", "0", "0", "0", "0", "0", "base_link", "zed_camera_link"]
    )

    tf_traj_gen = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world_ned", "traj_gen"]
    )

    # Publikacja opisu robota
    rov_state_publisher_node = Node(
        package='rov_description',
        executable='rov_state_publisher',
        parameters=[LaunchConfiguration('config')]
    )

    # Wizualizacja ustawienia ROV-a oraz działania pędników w RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen'
    )

    return LaunchDescription([
        config_arg,
        rviz_config_arg,
        ekf_node,
        pid_node,
        thruster_manager_node,
        # rov_bridge,
        joy_to_target_tf_node,
        joy_node,
        tf_zed,
        tf_traj_gen,
        TimerAction(period=1.0, actions=[
            rov_state_publisher_node,
            rviz_node
        ])
    ])