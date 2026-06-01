import math

from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose, Twist, WrenchStamped, TransformStamped, Transform
import numpy as np
import ros2_numpy as rnp
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster
import tf_transformations

np.printoptions(precision=4, suppress=True)


class PID(Node):

    def __init__(self):
        Node.__init__(self=self, node_name='rov_pid_node')

        self.declare_parameter('desired_position_topic_name', '/target_pose')
        self.declare_parameter('odom_topic_name', '/odometry/filtered')

        self.declare_parameter('odom_frame', 'base_link')
        self.declare_parameter('target_frame', 'traj_gen_node')
        self.declare_parameter('reference_frame', 'world_ned')
        self.declare_parameter('hz', 120)

        self.wrench_pub = self.create_publisher(WrenchStamped, 'calculated_wrench', 0)
        self.sync_trajectory_srv = self.create_service(Trigger, 'sync_trajectory', self.cb_sync_trajectory)

        self.tf_buffer = Buffer()
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.odom_clock = None
        self.target_clock = None
        self.main_clock = None

        self.odom_sub = None

        self.integral = np.zeros(6)
        self.prev_error = np.zeros(6)

        self.kp = [4.0, 4.0, 7.5, 0.1, 0.1, 0.1]#[4.0, 4.0, 7.5, 0.5, 0.0, 0.0]
        self.ki = [0.1, 0.1, 0.05, 0.001, 0.001, 0.001]#[0.1, 0.1, 0.05, 0.01, 0.0, 0.0]
        self.kd = [0.5, 0.5, 0.5, 0.05, 0.05, 0.05]#[0.5, 0.5, 0.5, 0.01, 0.0, 0.0]
        self.kp *= np.eye(6)
        self.ki *= np.eye(6)
        self.kd *= np.eye(6)

        self.__des_pos = np.zeros(6)
        self.__pos = np.zeros(6)

        self._frames_synced = False

        self.odom_clock = self.create_timer(1 / self.hz, self.cb_odom_frame)
        self.target_clock = self.create_timer(1 / self.hz, self.cb_target_frame)
        self.main_clock = self.create_timer(1 / self.hz, self.cb_main_clock)
        self.sync_frames()
        self.sync_clock = self.create_timer(1 / self.hz, self.cb_sync_clock)

    def cb_odom_frame(self) -> None:
        now = Time()
        try:
            t = self.tf_buffer.lookup_transform(self.odom_frame, self.reference_frame, now)
        except Exception as e:
            self.get_logger().error(f"{e}")
            return

        transform = rnp.numpify(t.transform)
        self.pos = transform

    def cb_target_frame(self) -> None:
        now = Time()
        try:
            t = self.tf_buffer.lookup_transform(self.target_frame, self.reference_frame, now)
        except Exception as e:
            self.get_logger().error(f"{e}")
            return

        transform = rnp.numpify(t.transform)
        self.des_pos = transform
        tf_transformations

    def cb_main_clock(self) -> None:
        if not self._frames_synced:
            return

        dt = 1 / self.hz

        try:
            # Pobieramy transformację bezpośrednio: z odom_frame (base_link) do target_frame
            t = self.tf_buffer.lookup_transform(self.odom_frame, self.target_frame, Time())

            # rnp.numpify zwraca macierz transformacji homogenicznej 4x4
            T = rnp.numpify(t.transform)

            # Wyciągamy translację (pozycja X, Y, Z celu w układzie robota)
            # To jest bezpośrednio nasz błąd translacji!
            error_pos = T[:3, 3]

            # Wyciągamy rotację i zamieniamy kwaternion na kąty Eulera
            # Najbezpieczniej bezpośrednio z macierzy rotacji 3x3 lub kwaternionu z wiadomości:
            quat = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
            error_rot = np.array(tf_transformations.euler_from_quaternion(quat))

            # Łączymy w jeden wektor błędu [X, Y, Z, Roll, Pitch, Yaw]
            error = np.concatenate((error_pos, error_rot), axis=None)

        except Exception as e:
            self.get_logger().error(f"Problem z pobraniem TF: {e}")
            return

        # Normalizacja kątów dla rotacji (indeksy 3, 4, 5)
        for i in range(3, 6):
            error[i] = (error[i] + np.pi) % (2 * np.pi) - np.pi
        error[2]+=0.1
        error[3]+= math.pi/2
        # Obliczenia PID (skoro error to Robot->Target, to znak jest już poprawny!)
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        out = (self.kp @ error) + (self.ki @ self.integral) + (self.kd @ derivative)

        # Budowanie wiadomości Wrench
        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = self.get_clock().now().to_msg()
        wrench_msg.header.frame_id = self.odom_frame  # 'base_link'

        wrench_msg.wrench.force.x = float(out[0])
        wrench_msg.wrench.force.y = float(out[1])
        wrench_msg.wrench.force.z = float(out[2])
        wrench_msg.wrench.torque.x = float(out[3])
        wrench_msg.wrench.torque.y = float(out[4])
        wrench_msg.wrench.torque.z = float(out[5])

        self.wrench_pub.publish(wrench_msg)
        self.prev_error = error.copy()

    def cb_sync_trajectory(self, request: Trigger.Request, response: Trigger.Response):
        ok, msg = self.sync_frames()
        response.success = bool(ok)
        response.message = msg
        return response

    def cb_sync_clock(self) -> None:
        if self.sync_frames()[0]:
            self._frames_synced = True
            self.sync_clock.cancel()

    def sync_frames(self) -> tuple[bool, str]:
        now = self.get_clock().now()
        target_parent_frame = 'traj_gen'
        try:
            t = self.tf_buffer.lookup_transform(target_parent_frame, self.target_frame, Time())
            t_base = self.tf_buffer.lookup_transform(self.reference_frame, self.odom_frame, Time())

            T = rnp.numpify(t.transform)
            T_base = rnp.numpify(t_base.transform)
            T = T_base @ np.linalg.inv(T)
            t.transform = rnp.msgify(Transform, T)

        except Exception as e:
            msg = f'Could not sync frames {self.odom_frame}, {self.target_frame}\n{e}'
            return False, msg

        tn = TransformStamped()
        tn.header.frame_id = self.reference_frame
        tn.header.stamp = now.to_msg()
        tn.child_frame_id = 'traj_gen'
        tn.transform = t.transform

        self.tf_broadcaster.sendTransform(tn)
        return True, 'Frames synced'

    @property
    def desired_position_topic_name(self):
        return self.get_parameter('desired_position_topic_name').value

    @property
    def odom_topic_name(self):
        return self.get_parameter('odom_topic_name').value

    @property
    def odom_frame(self) -> str:
        return self.get_parameter('odom_frame').value

    @property
    def target_frame(self) -> str:
        return self.get_parameter('target_frame').value

    @property
    def reference_frame(self) -> str:
        return self.get_parameter('reference_frame').value

    @property
    def hz(self) -> int:
        return self.get_parameter('hz').value

    @property
    def des_pos(self) -> np.ndarray:
        return self.__des_pos

    @des_pos.setter
    def des_pos(self, value: np.ndarray) -> None:
        self.__des_pos = value

    @property
    def pos(self) -> np.ndarray:
        return self.__pos

    @pos.setter
    def pos(self, value: np.ndarray) -> None:
        self.__pos = value

    @rnp.registry.converts_to_numpy(Twist)
    def convert(my_msg: Twist) -> np.ndarray:
        v = np.array([my_msg.linear.x, my_msg.linear.y, my_msg.linear.z], dtype=np.float64)
        w = np.array([my_msg.angular.z, my_msg.angular.y, my_msg.angular.x], dtype=np.float64)

        t = np.concatenate((v, w), axis=None)
        return t

    @rnp.registry.converts_from_numpy(Twist)
    def convert_back(arr: np.ndarray) -> Twist:
        msg = Twist()
        msg.linear.x = arr[0]
        msg.linear.y = arr[1]
        msg.linear.z = arr[2]
        msg.angular.x = arr[3]
        msg.angular.y = arr[4]
        msg.angular.z = arr[5]

        return msg
