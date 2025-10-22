import rclpy
from rov_position_snapshot.PositionSnapshot import PositionSnapshot


def main():
    rclpy.init()

    positionSnapshot = PositionSnapshot()

    rclpy.spin(positionSnapshot)

    positionSnapshot.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
