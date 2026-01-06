import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class EnableStonefishCurrents(Node):
    def __init__(self):
        super().__init__('enable_stonefish_currents')

        client = self.create_client(Trigger, '/stonefish_ros2/enable_currents')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

        req = Trigger.Request()

        future = client.call_async(req)
        future.add_done_callback(self.done)

    def done(self, future):
        try:
            result = future.result()
            self.get_logger().info(f"Service response: {result.success}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = EnableStonefishCurrents()
    rclpy.spin(node)
