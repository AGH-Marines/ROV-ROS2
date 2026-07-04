import numpy as np
import rclpy
from joblib.testing import param
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import (Vector3Stamped,
                               PoseStamped,
                               TwistStamped,
                               AccelStamped,
                               Point,
                               TransformStamped)
import py_trees
import py_trees_ros
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from autonomy_system.helpers.ControlOperations import ControlOperations
from autonomy_system_interfaces.action import NormalAction
from tf2_ros import TransformBroadcaster
import time
from rclpy.executors import MultiThreadedExecutor

class StabilizeOnPositionServer(Node):

    def __init__(self):
        super().__init__('stabilize_on_position_server')
        self.declare_parameter('action_name', "stabilize_on_position")
        self.declare_parameter('position_stabilization', [0.0, 0.0, 0.0])
        self.declare_parameter('rotation_stabilization', [0.0, 0.0, 0.0])
        self.declare_parameter('near_time', 5.0)
        self.declare_parameter('near_dist', 5.0)
        self._server = ActionServer(
            self,
            NormalAction,
            self.action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.control_operations = ControlOperations(self)

    @property
    def action_name(self):
        return self.get_parameter("action_name").value

    @property
    def position_stabilization(self):
        return np.array(self.get_parameter("position_stabilization").value)

    @property
    def rotation_stabilization(self):
        return np.array(self.get_parameter("rotation_stabilization").value)

    @property
    def near_time(self):
        return self.get_parameter("near_time").value

    @property
    def near_dist(self):
        return self.get_parameter("near_dist").value

    def execute_callback(self, goal_handle):
        self.get_logger().info("[Stabilize on Position] Started")

        feedback = NormalAction.Feedback()
        is_near = False
        near_start_time = time.time()
        while not (is_near and time.time() - near_start_time > self.near_time):
            position = self.position_stabilization
            att_rpy = self.rotation_stabilization
            print(self.control_operations.position(), position)
            print(np.linalg.norm(self.control_operations.position() - position), self.near_dist,is_near,time.time() - near_start_time)
            if np.linalg.norm(self.control_operations.position() - position) < self.near_dist:
                if not is_near:
                    is_near = True
                    near_start_time = time.time()
            else:
                is_near = False
            self.control_operations.pub_pose_cmd(position, att_rpy)
            self.control_operations.pub_transform_broadcaster(position, att_rpy)
            if goal_handle.is_cancel_requested:
                self.get_logger().warn("[Stabilize on Position] Cancelled")
                goal_handle.canceled()
                result = NormalAction.Result()
                result.success = False
                return result

            feedback.status = f"Stabilize..."
            goal_handle.publish_feedback(feedback)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.1)

        goal_handle.succeed()
        result = NormalAction.Result()
        result.success = True

        self.get_logger().info("[Stabilize on Position] Finished SUCCESS")
        return result

    def goal_callback(self, goal_request):
        self.get_logger().info("Received goal request")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn("Received cancel request")
        return CancelResponse.ACCEPT


def main():
    rclpy.init()

    node = StabilizeOnPositionServer()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
