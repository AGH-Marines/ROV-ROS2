import rclpy
from rov_controll_bus.ControllerBus import ControllerBus


def main():
    rclpy.init()

    node = ControllerBus()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
