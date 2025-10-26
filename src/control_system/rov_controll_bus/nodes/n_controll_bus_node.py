import rclpy
from rov_controll_bus.ControllerBus import ControllerBus
from rclpy.executors import MultiThreadedExecutor, Executor


def main():
    rclpy.init()
    node = ControllerBus()
    exec = MultiThreadedExecutor(num_threads=2)  # reentrant callbacks + service replies
    exec.add_node(node)
    try:
        exec.spin()
    except KeyboardInterrupt:
        pass
    finally:
        exec.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
