import os
import sys
import time
import unittest

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import launch_ros
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_testing.actions import ReadyToTest
import rclpy

def generate_test_description():
    controll_bus = FindPackageShare('rov_controll_bus')

    test_config_path = PathJoinSubstitution([controll_bus, 'config', 'test_load_params.yaml'])

    config_arg = DeclareLaunchArgument(
        name="config",
        default_value=test_config_path,
        description="Path to the configuration file for the stonefish simulator"
    )

    controll_bus_node = Node(
        package='rov_controll_bus',
        executable='controll_bus',
        parameters=[LaunchConfiguration('config')]
    )

    readyToTest = TimerAction(
        period=0.5, actions=[ReadyToTest()]
    )

    return (
        LaunchDescription([
            controll_bus_node,
            readyToTest
        ]),
        {
            'controll_bus': controll_bus_node
        }
    )


class TestControllerBus(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        self.node = rclpy.create_node('test_rov_controll_bus')

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_load_params(self, controll_bus, proc_output) -> None:
        self.node.get_logger().warning(f'{proc_output}')