import time

import py_trees
import py_trees_ros
import rclpy
from autonomy_system_interfaces.action import (
    SearchGate,
    MoveToGate,
    HelloWorld
)
from rclpy.action import ActionClient
from rclpy.node import Node


class ActionBehaviour(py_trees.behaviour.Behaviour):

    def __init__(
            self,
            name: str,
            node: Node,
            action_type,
            action_name: str,
            timeout_sec: float = 30.0,
    ):
        super().__init__(name)
        self.node = node
        self.timeout_sec = timeout_sec

        self.client = ActionClient(node, action_type, action_name)

        self._goal_future = None
        self._result_future = None
        self._start_time = None

    def initialise(self):
        self.node.get_logger().info(f"{self.name}: initialise")
        self._goal_future = None
        self._result_future = None
        self._start_time = None

    def update(self):

        if not self.client.wait_for_server(timeout_sec=0.1):
            self.node.get_logger().warn(
                f"{self.name}: action server not available"
            )
            return py_trees.common.Status.RUNNING

        if self._goal_future is None:
            self._start_time = self.node.get_clock().now()

            goal_msg = self.client._action_type.Goal()
            self._goal_future = self.client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback,
            )
            return py_trees.common.Status.RUNNING

        elapsed = (
                          self.node.get_clock().now() - self._start_time
                  ).nanoseconds / 1e9

        if elapsed > self.timeout_sec:
            self.node.get_logger().error(
                f"{self.name}: TIMEOUT"
            )
            return py_trees.common.Status.FAILURE

        if self._goal_future.done() and self._result_future is None:
            goal_handle = self._goal_future.result()

            if not goal_handle.accepted:
                self.node.get_logger().error(
                    f"{self.name}: goal rejected"
                )
                return py_trees.common.Status.FAILURE

            self._result_future = goal_handle.get_result_async()
            return py_trees.common.Status.RUNNING

        if self._result_future and self._result_future.done():
            result = self._result_future.result().result

            if result.success:
                self.node.get_logger().info(
                    f"{self.name}: SUCCESS"
                )
                return py_trees.common.Status.SUCCESS

            self.node.get_logger().error(
                f"{self.name}: FAILURE"
            )
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def feedback_callback(self, feedback_msg):
        self.node.get_logger().info(
            f"{self.name} feedback: {feedback_msg.feedback.status}"
        )


class MissionNode(Node):

    def __init__(self):
        super().__init__("mission_node")

        self.tree = self.create_tree()
        self.bt = py_trees_ros.trees.BehaviourTree(self.tree)
        self.bt.setup()

    def create_tree(self):
        root = py_trees.composites.Sequence(
            name="Mission",
            memory=True,
        )

        root.add_children([
            ActionBehaviour(
                name="Search Gate",
                node=self,
                action_type=SearchGate,
                action_name="search_gate"
            ),
            ActionBehaviour(
                name="Move To Gate",
                node=self,
                action_type=MoveToGate,
                action_name="move_to_gate"
            ),
            ActionBehaviour(
                name="Hello World",
                node=self,
                action_type=HelloWorld,
                action_name="hello_world"
            ),
        ])

        return root


def main():
    rclpy.init()

    node = MissionNode()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:

        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
            node.bt.tick()
            status = node.tree.status
            if status in (
                    py_trees.common.Status.SUCCESS,
                    py_trees.common.Status.FAILURE,
            ):
                node.get_logger().info(
                    f"Mission finished with status: {status}"
                )
                break
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
