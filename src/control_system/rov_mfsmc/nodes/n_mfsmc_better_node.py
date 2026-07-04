import rclpy
from rov_mfsmc.ModelFreeSlidingControl_Better import ModelFreeSlidingControl


def main():
    rclpy.init()

    node = ModelFreeSlidingControl()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
