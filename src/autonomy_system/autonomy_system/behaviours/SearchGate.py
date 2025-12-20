import rclpy
import py_trees
import py_trees_ros
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
import time
from autonomy_system_interfaces.action import SearchGate, MoveToGate, HelloWorld


class SearchGateServer(Node):

    def __init__(self):
        super().__init__('search_gate_server')
        self._server = ActionServer(
            self,
            SearchGate,
            "search_gate",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info("[SearchGate] Started")

        feedback = SearchGate.Feedback()

        for i in range(10):
            if goal_handle.is_cancel_requested:
                self.get_logger().warn("[SearchGate] Cancelled")
                goal_handle.canceled()
                result = SearchGate.Result()
                result.success = False
                return result

            feedback.status = f"Searching gate... {i + 1}/10"
            goal_handle.publish_feedback(feedback)
            time.sleep(1)

        goal_handle.succeed()
        result = SearchGate.Result()
        result.success = True

        self.get_logger().info("[SearchGate] Finished SUCCESS")
        return result

    def goal_callback(self, goal_request):
        self.get_logger().info("Received goal request")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn("Received cancel request")
        return CancelResponse.ACCEPT


def main():
    rclpy.init()

    node = SearchGateServer()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
