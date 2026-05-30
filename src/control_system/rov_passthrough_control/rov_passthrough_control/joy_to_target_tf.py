import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import tf_transformations
import numpy as np


class JoyToTargetTF(Node):
    def __init__(self):
        super().__init__('joy_to_target_tf')

        self.declare_parameter('parent_frame', 'world_ned')
        self.declare_parameter('target_frame', 'traj_gen_node')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('hz', 50.0)

        # Prędkości maksymalne
        self.declare_parameter('max_lin_vel', 0.5)  # m/s
        self.declare_parameter('max_ang_vel', 0.5)  # rad/s

        self.parent_frame = self.get_parameter('parent_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.hz = self.get_parameter('hz').value
        self.max_lin_vel = self.get_parameter('max_lin_vel').value
        self.max_ang_vel = self.get_parameter('max_ang_vel').value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        self.timer = self.create_timer(1.0 / self.hz, self.update_callback)

        self.target_pose = np.zeros(6)  # x, y, z, roll, pitch, yaw
        self.joy_input = np.zeros(6)
        self.initialized = False

    def joy_callback(self, msg):
        # Mapowanie osi (zakładając standardowego pada DS4 przez joy node)
        # Lewy drążek: x (przód/tył), y (lewo/prawo)
        # Prawy drążek: yaw (obrót), z (góra/dół)

        # Uwaga: Mapowanie może wymagać dostosowania w zależności od konfiguracji joy
        if len(msg.axes) >= 6:
            self.joy_input[0] = msg.axes[1]  # x
            self.joy_input[1] = msg.axes[0]  # y
            self.joy_input[2] = msg.axes[4]  # z (często R2/L2 lub prawy drążek)
            self.joy_input[5] = msg.axes[3]  # yaw (prawy drążek x)

    def update_callback(self):
        # Nie musimy szukać TF ani całkować!
        # Zamiast powiększać pozycję (+=), po prostu rzutujemy
        # bezpośrednio wejście z pada na dystans od robota (=).

        target_x = self.joy_input[0] * self.max_lin_vel
        target_y = self.joy_input[1] * self.max_lin_vel
        target_z = self.joy_input[2] * self.max_lin_vel
        target_yaw = self.joy_input[5] * self.max_ang_vel

        # Publikacja TF "Marchewki"
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame  # Czyli teraz 'base_link'
        t.child_frame_id = self.target_frame  # 'traj_gen_node'

        t.transform.translation.x = target_x
        t.transform.translation.y = target_y
        t.transform.translation.z = target_z

        # Obrót z kwaternionów (tylko odchylenie Yaw na prawym drążku)
        quat = tf_transformations.quaternion_from_euler(0.0, 0.0, target_yaw)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = JoyToTargetTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()