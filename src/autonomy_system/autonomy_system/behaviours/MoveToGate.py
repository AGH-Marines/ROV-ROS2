import rclpy
import py_trees
import py_trees_ros
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse

from autonomy_system_interfaces.action import SearchGate, MoveToGate, HelloWorld
import time


class MoveToGateServer(Node):

    def __init__(self):
        super().__init__('move_to_gate_server')
        self._server = ActionServer(
            self,
            MoveToGate,
            "move_to_gate",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info("[MoveToGate] Started")

        feedback = MoveToGate.Feedback()

        for i in range(10):
            if goal_handle.is_cancel_requested:
                self.get_logger().warn("[MoveToGate] Cancelled")
                goal_handle.canceled()
                result = MoveToGate.Result()
                result.success = False
                return result

            feedback.status = f"Moving to gate... {i+1}/10"
            goal_handle.publish_feedback(feedback)
            time.sleep(1)

        goal_handle.succeed()
        result = MoveToGate.Result()
        result.success = True

        self.get_logger().info("[MoveToGate] Finished SUCCESS")
        return result
    def goal_callback(self, goal_request):
        self.get_logger().info("Received goal request")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn("Received cancel request")
        return CancelResponse.ACCEPT


def main():
    rclpy.init()

    node = MoveToGateServer()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
