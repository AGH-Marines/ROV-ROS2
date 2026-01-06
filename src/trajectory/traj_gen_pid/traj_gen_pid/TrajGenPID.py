from rclpy.node import Node
from rclpy.time import Time
from dataclasses import dataclass
import re
from datetime import datetime
import os
import json
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, Pose, PoseStamped, Transform
from nav_msgs.msg import Path
from rosidl_runtime_py import message_to_ordereddict
import time

@dataclass
class Snapshot:
    pose: Pose
    time: float


class TrajGenPID(Node):

    def __init__(self):
        Node.__init__(self=self, node_name="traj_gen_pid")

        self.declare_parameter('frame_id', 'traj_gen')
        self.declare_parameter('child_frame_id', 'traj_gen_node')

        self.declare_parameter('use_traj_from_file', True)
        self.declare_parameter('traj_file', 'output.json')
        self.declare_parameter('traj_file_dir', '/home/dev/ros2_ws/src/trajectory/rov_position_snapshot/output')
        self.declare_parameter('hz', 5)

        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value

        self.use_traj_from_file = self.get_parameter('use_traj_from_file').value
        self.traj_file = self.get_parameter('traj_file').value
        self.traj_file_dir = self.get_parameter('traj_file_dir').value
        self.hz = self.get_parameter('hz').value

        self.snapshots: list[Snapshot] = []

        self.load_trajectory()

        self.tf_buffer = Buffer()
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.main_clock = self.create_timer(1.0 / self.hz, self.main_clock_cb)

        self.start_time = self.get_clock().now()

        transform = TransformStamped()
        transform.header.frame_id = self.frame_id
        transform.header.stamp = self.start_time.to_msg()
        transform.child_frame_id = self.child_frame_id
        transform.transform.translation.x = self.snapshots[0].pose.position.x
        transform.transform.translation.y = self.snapshots[0].pose.position.y
        transform.transform.translation.z = self.snapshots[0].pose.position.z
        transform.transform.rotation.x = self.snapshots[0].pose.orientation.x
        transform.transform.rotation.y = self.snapshots[0].pose.orientation.y
        transform.transform.rotation.z = self.snapshots[0].pose.orientation.z
        transform.transform.rotation.w = self.snapshots[0].pose.orientation.w

        self.tf_broadcaster.sendTransform(transform)

    def main_clock_cb(self):
        now = self.get_clock().now()
        dt = (now - self.start_time).nanoseconds * 1e-9

        snapshot = self.snapshots[1]

        if dt < snapshot.time:
            return

        self.snapshots.pop(0)
        snapshot = self.snapshots[0]

        transform = TransformStamped()
        transform.header.frame_id = self.frame_id
        transform.header.stamp = now.to_msg()
        transform.child_frame_id = self.child_frame_id
        transform.transform.translation.x = snapshot.pose.position.x
        transform.transform.translation.y = snapshot.pose.position.y
        transform.transform.translation.z = snapshot.pose.position.z
        transform.transform.rotation.x = snapshot.pose.orientation.x
        transform.transform.rotation.y = snapshot.pose.orientation.y
        transform.transform.rotation.z = snapshot.pose.orientation.z
        transform.transform.rotation.w = snapshot.pose.orientation.w

        self.tf_broadcaster.sendTransform(transform)

    def load_trajectory(self):
        file_path = os.path.join(self.traj_file_dir, self.traj_file)

        with open(file_path, 'r') as file:
            data = json.load(file)

        time_ratio = data['time_ratio']

        for point in data['data']:
            pose = Pose()
            pose.position.x = point['position']['x']
            pose.position.y = point['position']['y']
            pose.position.z = point['position']['z']
            pose.orientation.x = point['orientation']['x']
            pose.orientation.y = point['orientation']['y']
            pose.orientation.z = point['orientation']['z']
            pose.orientation.w = point['orientation']['w']

            snapshot = Snapshot(pose=pose, time=point['time'] * time_ratio)

            self.snapshots.append(snapshot)
