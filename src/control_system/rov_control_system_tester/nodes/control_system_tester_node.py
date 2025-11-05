import rclpy
from rov_control_system_tester.ControlSystemTester import ControlSystemTester


def main():
    rclpy.init()

    node = ControlSystemTester()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
