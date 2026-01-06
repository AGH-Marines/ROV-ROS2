import rclpy
from traj_gen_pid.TrajGenPID import TrajGenPID


def main():
    rclpy.init()

    trajGenPID = TrajGenPID()

    rclpy.spin(trajGenPID)

    trajGenPID.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
