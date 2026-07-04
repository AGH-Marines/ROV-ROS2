import numpy as np
import py_trees
import py_trees_ros
import rclpy
from autonomy_system_interfaces.action import NormalAction
from geometry_msgs.msg import (Vector3Stamped,
                               PoseStamped,
                               TwistStamped,
                               AccelStamped,
                               Point,
                               TransformStamped)
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from tf2_ros import TransformBroadcaster, TransformListener, StaticTransformBroadcaster
import ros2_numpy as rnp
from rclpy.time import Time
from tf_transformations import euler_from_matrix
from tf2_ros import Buffer


class ControlOperations:
    def __init__(self, node):
        self.node = node
        self.pose_sp_pub = node.create_publisher(PoseStamped, '/target_pose', 10)
        self.transform_broadcaster = TransformBroadcaster(node)
        self.frame_id_param = node.declare_parameter('frame_id', 'traj_gen')
        self.child_frame_id_param = node.declare_parameter('child_frame_id', 'traj_gen_node')
        node.declare_parameter('odom_frame', 'base_link')
        node.declare_parameter('reference_frame', 'world_ned')
        node.declare_parameter('hz', 120)

        self.tf_buffer = Buffer()
        self.tf_broadcaster = StaticTransformBroadcaster(node)
        self.tf_listener = TransformListener(self.tf_buffer, node)
        self.odom_clock = self.node.create_timer(1 / self.hz, self.cb_odom_frame)

        self.pos = np.array([1000.0, 1000.0, 1000.0])
        self.rot = np.array([0.0, 0.0, 0.0])

    def position(self):
        return self.pos

    def rotation(self):
        return self.rot

    def cb_odom_frame(self) -> None:
        now = Time()
        try:
            t = self.tf_buffer.lookup_transform(self.reference_frame,self.odom_frame,  now)
        except Exception as e:
            self.node.get_logger().error(f"{e}")
            return
        transform = rnp.numpify(t.transform)
        pos_from_tf = transform[:3, 3]
        rotation_matrix = transform[:3, :3]  # 3x3
        roll, pitch, yaw = euler_from_matrix(rotation_matrix)
        self.pos = pos_from_tf
        self.rot = np.array([roll, pitch, yaw])

    @property
    def child_frame_id(self) -> str:
        return self.node.get_parameter('child_frame_id').value

    @property
    def frame_id(self) -> str:
        return self.node.get_parameter('frame_id').value

    @property
    def odom_frame(self) -> str:
        return self.node.get_parameter('odom_frame').value

    @property
    def reference_frame(self) -> str:
        return self.node.get_parameter('reference_frame').value

    @property
    def hz(self) -> int:
        return self.node.get_parameter('hz').value

    def pub_pose_cmd(self, position: np.array, rpy: np.array) -> PoseStamped:
        q_tmp = Rotation.from_euler(
            'XYZ', [rpy[0], rpy[1], rpy[2]]).as_quat()
        q = np.zeros(4)
        q[0] = q_tmp[3]
        q[1] = q_tmp[0]
        q[2] = q_tmp[1]
        q[3] = q_tmp[2]
        msg = PoseStamped()
        msg.header.frame_id = self.child_frame_id
        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
        msg.pose.position.z = position[2]
        msg.pose.orientation.w = q[0]
        msg.pose.orientation.x = q[1]
        msg.pose.orientation.y = q[2]
        msg.pose.orientation.z = q[3]
        self.pose_sp_pub.publish(msg)
        return msg

    def pub_transform_broadcaster(self, position: np.ndarray, rpy: np.ndarray) -> None:
        q_tmp = Rotation.from_euler(
            'XYZ', [rpy[0], rpy[1], rpy[2]]
        ).as_quat()
        q = np.zeros(4)
        q[0] = q_tmp[3]
        q[1] = q_tmp[0]
        q[2] = q_tmp[1]
        q[3] = q_tmp[2]
        msg = TransformStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.child_frame_id
        msg.transform.translation.x = position[0]
        msg.transform.translation.y = position[1]
        msg.transform.translation.z = position[2]
        msg.transform.rotation.w = q[0]
        msg.transform.rotation.x = q[1]
        msg.transform.rotation.y = q[2]
        msg.transform.rotation.z = q[3]

        self.transform_broadcaster.sendTransform(msg)


def pub_transform_broadcaster(self, position: np.ndarray, rpy: np.ndarray) -> None:
    q_tmp = Rotation.from_euler(
        'XYZ', [rpy[0], rpy[1], rpy[2]]
    ).as_quat()
    q = np.zeros(4)
    q[0] = q_tmp[3]
    q[1] = q_tmp[0]
    q[2] = q_tmp[1]
    q[3] = q_tmp[2]
    msg = TransformStamped()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.header.frame_id = self.frame_id
    msg.child_frame_id = self.child_frame_id
    msg.transform.translation.x = position[0]
    msg.transform.translation.y = position[1]
    msg.transform.translation.z = position[2]
    msg.transform.rotation.w = q[0]
    msg.transform.rotation.x = q[1]
    msg.transform.rotation.y = q[2]
    msg.transform.rotation.z = q[3]

    self.transform_broadcaster.sendTransform(msg)
