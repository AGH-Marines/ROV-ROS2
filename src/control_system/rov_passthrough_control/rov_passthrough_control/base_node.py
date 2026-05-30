import rclpy
from rov_passthrough_control.PassthroughControl import PassthroughControl


def main():
    rclpy.init()

    passthroughControl = PassthroughControl()

    rclpy.spin(passthroughControl)

    passthroughControl.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
