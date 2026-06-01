from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Udziały pakietów
    launchers_share = FindPackageShare('launchers')
    pid_share = FindPackageShare('rov_pid')
    thruster_manager_share = FindPackageShare('rov_thruster_manager')

    # Domyślne ścieżki konfiguracji
    default_config_path = PathJoinSubstitution([pid_share, 'config', 'params.yaml'])
    default_rviz_config_path = PathJoinSubstitution([thruster_manager_share, 'rviz', 'thruster.rviz'])

    # Ścieżka do launcha ZED
    zed_wrapper_share = FindPackageShare('zed_wrapper')
    zed_launch_file = PathJoinSubstitution([zed_wrapper_share, 'launch', 'zed_camera.launch.py'])

    # Uruchomienie ZED - Skonfigurowane tak, aby wpiąć się bezpośrednio w base_link robota
    zed_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_launch_file),
        launch_arguments={
            'camera_model': 'zed2i',
            'publish_tf': 'true',            # Włączone, aby ZED sam zbudował drzewo do base_link
            'base_frame_id': 'base_link',     # Główna rama Twojego ROV-a
            'map_frame_id': 'world_ned',      # Układ globalny zgodny z Twoim PID
            'odom_frame_id': 'odom',
            'publish_odometry_tf': 'true',
            'set_as_static': 'false',
            'publish_imu_tf': 'true'
        }.items()
    )

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

    # Węzeł Zarządzania Pędnikami (Zabezpieczony automatycznym restartem w razie math domain error)
    thruster_manager_node = Node(
        package='rov_thruster_manager',
        executable="thruster_manager",
        parameters=[LaunchConfiguration('config')],
        output="screen",
        respawn=True,
        respawn_delay=2.0
    )

    # Węzeł mostka (rov_bridge) - komunikacja z mikrokontrolerem
    rov_bridge = Node(
        package="rov_bridge",
        executable='uard',
        parameters=[LaunchConfiguration('config')],
        output="screen"
    )

    # Węzeł konwersji Joystick -> Cel TF (Poprawiona nazwa ramy docelowej na 'traj_gen')
    joy_to_target_tf_node = Node(
        package='rov_passthrough_control',
        executable='joy_to_target_tf',
        parameters=[{
            'parent_frame': 'world_ned',
            'target_frame': 'traj_gen_node',       # <--- POPRAWIONE (było traj_gen_node)
            'base_frame': 'base_link',
            'max_lin_vel': 0.3,
            'max_ang_vel': 0.3
        }],
        output='screen'
    )

    # Driver pada
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'deadzone': 0.1}]
    )

    # Statyczna transformacja dla generatora trajektorii
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

    # Wizualizacja w RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen'
    )

    tf_zed = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        # arguments: x, y, z, yaw, pitch, roll, parent, child
        # Zakładamy, że kamera jest wysunięta np. o 20cm w przód (X) względem środka drona.
        # Jeśli base_link względem kamery jest przesunięty w tył, dajemy -0.2 na osi X.
        arguments=["-0.2", "0", "0", "0", "0", "0", "zed_camera_link", "base_link"]
    )

    return LaunchDescription([
        config_arg,
        rviz_config_arg,
        ekf_node,
        pid_node,
        thruster_manager_node,
        rov_bridge,
        zed_camera,
        joy_to_target_tf_node,
        # joy_node,
        tf_zed,
        tf_traj_gen,  # Usunąłem stąd tf_zed, ponieważ 'base_link' jest teraz obsługiwany bezpośrednio przez wrapper ZEDa
        TimerAction(period=2.0, actions=[   # Zwiększyłem delikatnie opóźnienie, żeby dać Jetsonowi czas na inicjalizację GPU dla ZEDa
            rov_state_publisher_node,
            # rviz_node
        ])
    ])