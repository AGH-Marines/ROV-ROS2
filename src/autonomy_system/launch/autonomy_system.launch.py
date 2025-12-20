from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    hello_world_action = Node(
        package='autonomy_system',
        executable='hello_world_action',
        name='hello_world_action',
        output='screen',
        emulate_tty=True
    )

    move_to_gate_action = Node(
        package='autonomy_system',
        executable='move_to_gate_action',
        name='move_to_gate_action',
        output='screen',
        emulate_tty=True
    )

    search_gate_action = Node(
        package='autonomy_system',
        executable='search_gate_action',
        name='search_gate_action',
        output='screen',
        emulate_tty=True
    )

    behavior_tree = Node(
        package='autonomy_system',
        executable='behavior_tree',
        name='behavior_tree',
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        hello_world_action,
        move_to_gate_action,
        search_gate_action,
        behavior_tree,
    ])
