import rclpy
import py_trees
import py_trees_ros
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse

from autonomy_system_interfaces.action import SearchGate, MoveToGate, HelloWorld

class HelloWorldServer(Node):

    def __init__(self):
        super().__init__('hello_world_server')
        self._server = ActionServer(
            self,
            HelloWorld,
            "hello_world",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )


    def execute_callback(self, goal_handle):
        self.get_logger().info("[HelloWorld] HELLO WORLD 👋")

        goal_handle.succeed()
        result = HelloWorld.Result()
        result.success = True
        return result

    def goal_callback(self, goal_request):
        self.get_logger().info("Received goal request")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn("Received cancel request")
        return CancelResponse.ACCEPT



def main():
    rclpy.init()

    node = HelloWorldServer()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
