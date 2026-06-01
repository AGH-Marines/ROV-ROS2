from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

from ament_index_python import get_package_share_directory


def generate_launch_description():

    launchers_share = FindPackageShare('launchers')
    wrench_system_share = FindPackageShare('rov_passthrough_control')

    # Define default configuration paths
    default_config_path = PathJoinSubstitution([launchers_share, 'config', 'wreckage_bluerov2.yaml'])
    default_rviz_config_path = PathJoinSubstitution([launchers_share, 'rviz', 'tank_bluerov2_imu.rviz'])

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

    # Nodes and launch inclusions
    thruster_manager_node = Node(
        package='rov_thruster_manager',
        executable="thruster_manager",
        parameters=[LaunchConfiguration('config')],
        output="screen"
    )

    # wrench_system_launch = IncludeLaunchDescription(
    #     launch_description_source=PathJoinSubstitution([wrench_system_share, 'launch', 'base.launch.py']),
    #     launch_arguments={
    #         "config": LaunchConfiguration('config')
    #     }.items()
    # )

    rov_passthrough_control = Node(
        package="rov_passthrough_control",
        executable='base_node',
        parameters=[LaunchConfiguration('config')],
        output="screen"
    )

#    rov_bridge = Node(
#        package="rov_bridge",
#        executable='uard',
#        parameters=[LaunchConfiguration('config')],
#        output="screen"
#    )

    tf_imu = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "base_link",
            "--child-frame-id", "bluerov2/imu_filter"
        ]
    )


    tf_multibeam = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "bluerov2/fls"]
    )
    rov_state_publisher_node = Node(package='rov_description',
                                    executable='rov_state_publisher',
                                    parameters=[LaunchConfiguration('config')])
    description_timer = TimerAction(period=5.0, actions=[rov_state_publisher_node])
    return LaunchDescription([
        config_arg,
        rviz_config_arg,
 #       rov_bridge,
        thruster_manager_node,
        rov_passthrough_control,
        tf_imu,
        tf_multibeam,
        description_timer
    ])
