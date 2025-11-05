import rclpy
from rov_pid.PID import PID


def main():
    rclpy.init()

    node = PID()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
