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
        if not self.initialized:
            try:
                # Inicjalizacja celu w aktualnej pozycji robota
                t = self.tf_buffer.lookup_transform(self.parent_frame, self.base_frame, rclpy.time.Time())
                self.target_pose[0] = t.transform.translation.x
                self.target_pose[1] = t.transform.translation.y
                self.target_pose[2] = t.transform.translation.z

                quat = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
                euler = tf_transformations.euler_from_quaternion(quat)
                self.target_pose[3:] = euler
                self.initialized = True
                self.get_logger().info('Initialized target pose to current robot pose.')
            except Exception as e:
                self.get_logger().warn(f'Waiting for transform from {self.parent_frame} to {self.base_frame}: {e}')
                self.target_pose = np.zeros(6)
                self.initialized = True

        dt = 1.0 / self.hz

        # Obliczenie przemieszczeń lokalnych na podstawie joysticka
        dx_local = self.joy_input[0] * self.max_lin_vel * dt  # Przód / Tył
        dy_local = self.joy_input[1] * self.max_lin_vel * dt  # Lewo / Prawo
        dz_global = self.joy_input[2] * self.max_lin_vel * dt  # Góra / Dół (zostaje globalnie)
        dyaw = self.joy_input[5] * self.max_ang_vel * dt  # Obrót (Yaw)

        # Pobranie aktualnego kąta Yaw naszego celu
        current_yaw = self.target_pose[5]

        # Przeliczenie ruchu lokalnego na globalne osie X i Y (Rotacja 2D)
        dx_global = dx_local * np.cos(current_yaw) - dy_local * np.sin(current_yaw)
        dy_global = dx_local * np.sin(current_yaw) + dy_local * np.cos(current_yaw)

        # Aktualizacja pozycji celu (target_pose)
        self.target_pose[0] += dx_global
        self.target_pose[1] += dy_global
        self.target_pose[2] += dz_global
        self.target_pose[5] += dyaw

        # Publikacja TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.target_frame

        t.transform.translation.x = self.target_pose[0]
        t.transform.translation.y = self.target_pose[1]
        t.transform.translation.z = self.target_pose[2]

        quat = tf_transformations.quaternion_from_euler(
            self.target_pose[3], self.target_pose[4], self.target_pose[5])
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