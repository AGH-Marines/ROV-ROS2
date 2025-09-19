import rclpy
from rclpy.node import Node

from stonefish_ros2.msg import DVL
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped

import numpy as np


class DVLPoseTranslator(Node):
    def __init__(self):
        Node.__init__(self=self, node_name='dvl_pose_translation')
        self.declare_parameter('vehicle_name', 'bluerov2')
        self.vehicle_name = self.get_parameter('vehicle_name').value

        self.declare_parameter('dvl_topic_name', f"/{self.vehicle_name}/dvl_sim")
        self.dvl_topic_name = self.get_parameter('dvl_topic_name').value

        self.declare_parameter('pose_topic_name', f"/{self.vehicle_name}/dvl_pose")
        self.pose_topic_name = self.get_parameter('pose_topic_name').value

        self.declare_parameter('twist_topic_name', f"/{self.vehicle_name}/dvl_twist")
        self.twist_topic_name = self.get_parameter('twist_topic_name').value

        self.declare_parameter('dvl_altitude_noise', 0.001)
        self.dvl_altitude_noise = self.get_parameter('dvl_altitude_noise').value

        self.create_subscription(DVL, self.dvl_topic_name, self.cb_dvl_input, 10)
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, self.pose_topic_name, 10)
        self.pub_twist = self.create_publisher(TwistWithCovarianceStamped, self.twist_topic_name, 10)
        self.i = 0

    def cb_dvl_input(self, msg: DVL):

        pose_msg = PoseWithCovarianceStamped()
        twist_msg = TwistWithCovarianceStamped()

        pose_msg.header = msg.header
        twist_msg.header = msg.header

        altitude = msg.altitude
        velocity = msg.velocity

        if self.i == 0:
            velocity_covariance = msg.velocity_covariance[::4]

            empty_arr = np.zeros(6)
            empty_arr[0:3] = velocity_covariance
            eye = np.eye(6)

            velocity_covariance = eye * empty_arr
            velocity_covariance = velocity_covariance.flatten().tolist()
            self.vel_conv = velocity_covariance

            pose_convariance = self.dvl_altitude_noise * self.dvl_altitude_noise

            empty_arr = np.zeros(6)
            empty_arr[2] = pose_convariance
            eye = np.eye(6)

            pose_convariance = eye * empty_arr
            pose_convariance = pose_convariance.flatten().tolist()
            self.pose_conv = pose_convariance

            self.i += 1

        pose_msg.pose.pose.position.z = altitude
        pose_msg.pose.covariance = self.pose_conv

        twist_msg.twist.twist.linear.x = velocity.x
        twist_msg.twist.twist.linear.y = velocity.y
        twist_msg.twist.twist.linear.z = velocity.z
        twist_msg.twist.covariance = self.vel_conv

        self.pub_pose.publish(pose_msg)
        self.pub_twist.publish(twist_msg)


def main():
    rclpy.init()

    node = DVLPoseTranslator()

    rclpy.spin(node=node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
