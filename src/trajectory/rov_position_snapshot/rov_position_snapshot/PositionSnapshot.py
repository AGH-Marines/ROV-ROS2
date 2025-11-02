from rclpy.node import Node
from rclpy.time import Time
import re
from datetime import datetime
import os
import json
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, Pose, PoseStamped
from nav_msgs.msg import Path
from rosidl_runtime_py import message_to_ordereddict
import time

from ds4_driver_msgs.msg import Status


class PositionSnapshot(Node):
    def __init__(self):
        Node.__init__(self=self, node_name='rov_position_snapshot')

        self.declare_parameter('output_dir', '/home/dev/ros2_ws/src/trajectory/rov_position_snapshot/output')
        self.declare_parameter('filename', 'output_trajectory_')
        self.declare_parameter('ds4_snapshot_key', 'button_cross')
        self.declare_parameter('ds4_save_key', 'button_triangle')
        self.declare_parameter('parent_frame', 'world_ned')

        self.output_dir = self.get_parameter('output_dir').value
        self.filename = self.get_parameter('filename').value

        self.ds4_snapshot_key = self.get_parameter('ds4_snapshot_key').value
        self.ds4_save_key = self.get_parameter('ds4_save_key').value

        self.parent_frame = self.get_parameter('parent_frame').value

        self.sub_ds4 = self.create_subscription(Status, 'status', self.cb_ds4, 0)

        self.pub_waypoints = self.create_publisher(Path, 'position_snapshot/waypoints', 0)

        self.snapshots: list[PoseStamped] = []

        self.node_parent_frame_id = 'position_snapshot_base'
        self.node_child_frame_id = 'position_snapshot_node'

        self.tf_buffer = Buffer()
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def cb_ds4(self, msg: Status):

        if msg.__getattribute__(self.ds4_snapshot_key):
            self.take_snapshot()
            time.sleep(0.5)

        if msg.__getattribute__(self.ds4_save_key):
            self.save()
            time.sleep(0.5)

    def take_snapshot(self):
        self.get_logger().info('Taking Snapshot')

        now = self.get_clock().now()

        # 1) Jeśli to pierwszy snapshot – zakotwicz ramę „rodzica” w pozycji base_link względem parent_frame
        if len(self.snapshots) == 0:
            t = self.tf_buffer.lookup_transform(
                target_frame=self.parent_frame,
                source_frame='base_link',
                time=Time()
            )

            node_parent_frame = TransformStamped()
            node_parent_frame.header.frame_id = self.parent_frame
            node_parent_frame.header.stamp = now.to_msg()
            node_parent_frame.child_frame_id = self.node_parent_frame_id
            node_parent_frame.transform = t.transform

            self.tf_broadcaster.sendTransform(node_parent_frame)

            t_node = TransformStamped()
            t_node.header.frame_id = self.node_parent_frame_id
            t_node.header.stamp = now.to_msg()
            t_node.child_frame_id = self.node_child_frame_id
            t_node.transform.translation.x = 0.0
            t_node.transform.translation.y = 0.0
            t_node.transform.translation.z = 0.0
            t_node.transform.rotation.x = 0.0
            t_node.transform.rotation.y = 0.0
            t_node.transform.rotation.z = 0.0
            t_node.transform.rotation.w = 1.0

        else:
            t_lookup = self.tf_buffer.lookup_transform(
                target_frame=self.node_parent_frame_id,
                source_frame='base_link',
                time=Time()
            )

            t_node = TransformStamped()
            t_node.header.frame_id = self.node_parent_frame_id
            t_node.header.stamp = now.to_msg()
            t_node.child_frame_id = self.node_child_frame_id
            t_node.transform = t_lookup.transform

        self.tf_broadcaster.sendTransform(t_node)

        pose = Pose()
        pose.position.x = t_node.transform.translation.x
        pose.position.y = t_node.transform.translation.y
        pose.position.z = t_node.transform.translation.z
        pose.orientation = t_node.transform.rotation

        pose_stmp = PoseStamped()
        pose_stmp.header.frame_id = self.node_parent_frame_id
        pose_stmp.header.stamp = now.to_msg()
        pose_stmp.pose = pose

        self.snapshots.append(pose_stmp)

        path = Path()
        path.header.frame_id = self.node_parent_frame_id
        path.header.stamp = now.to_msg()
        path.poses = list(self.snapshots)
        self.pub_waypoints.publish(path)

    def save(self):
        now = datetime.now()
        filename = self.filename
        if not re.match(r'.*\.json', self.filename):
            filename += now.strftime("%m-%d_%H:%M")
            filename += '.json'

        dt = 0
        snapshots_dict = []

        for i, s in enumerate(self.snapshots):
            if i == 0:
                dt = self.stamp_to_datetime(s.header.stamp)
                dt = dt.timestamp()

            t = self.stamp_to_datetime(s.header.stamp)
            t = t.timestamp()

            d = {
                'position': message_to_ordereddict(s.pose.position),
                'orientation': message_to_ordereddict(s.pose.orientation),
                'time': t - dt
            }

            snapshots_dict.append(d)

        data = dict()
        data['data'] = snapshots_dict
        data['timestamp'] = now.timestamp()
        data['creation_time'] = now.strftime("%y-%m-%d_%H:%M")

        output_path = os.path.join(self.output_dir, filename)

        with open(output_path, 'w') as json_file:
            json.dump(data, json_file, indent=4)

        self.get_logger().info(f'Trajectory saved to file: {self.filename}')
        self.snapshots = []

    @staticmethod
    def stamp_to_datetime(stamp):
        # Combine sec + nanosec into one floating-point seconds value
        timestamp = stamp.sec + stamp.nanosec * 1e-9
        # Convert to UTC datetime
        return datetime.fromtimestamp(timestamp)
