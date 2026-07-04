from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose, Twist, TwistStamped, WrenchStamped, TransformStamped, Transform
import numpy as np
import ros2_numpy as rnp
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster
import tf_transformations

np.printoptions(precision=4, suppress=True)


class ModelFreeSlidingControl(Node):
    _is_running: bool = False

    def __init__(self):
        Node.__init__(self=self, node_name='rov_mfsmc_node')

        self.declare_parameter('desired_position_topic_name', '/target_pose')
        self.declare_parameter('desired_twist_topic_name', '/target_twist')
        self.declare_parameter('odom_topic_name', '/odometry/filtered')

        self.declare_parameter('A', [2.3, 2.3, 5.0, 0.1, 0.1, 0.1])
        self.declare_parameter('kd', [4.5, 4.5, 2.5, 0.1, 0.1, 0.1])
        self.declare_parameter('ki', [0.1, 0.2, 0.3, 0.0, 0.0, 0.0])
        self.declare_parameter('alpha', 0.15)
        self.declare_parameter('phi', [0.3, 0.3, 0.15, 0.4, 0.4, 0.4])
        self.declare_parameter('k_min', [0.6, 0.6, 0.6, 0.6, 0.6, 0.6])
        self.declare_parameter('k_max', [1.0, 1.0, 1.0, 1.0, 0.4, 1.0])
        self.declare_parameter('e_big', [0.5, 0.5, 0.5, 0.5, 0.5, 0.5])

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

        self.desired_twist_sub = None
        self.odom_sub = None

        self.__des_pos = np.zeros(6)
        self.__pos = np.zeros(6)
        self.__des_twist = np.zeros(6)
        self.__twist = np.zeros(6)

        self.u_sign = np.zeros(6)

        self._frames_synced = False

        self.desired_twist_sub = self.create_subscription(TwistStamped,
                                                          self.desired_twist_topic_name,
                                                          self.cb_desired_twist, 0)
        self.odom_sub = self.create_subscription(Odometry,
                                                 self.odom_topic_name,
                                                 self.cb_odom, qos_profile=0)

        self.odom_clock = self.create_timer(1 / self.hz, self.cb_odom_frame)
        self.target_clock = self.create_timer(1 / self.hz, self.cb_target_frame)
        self.main_clock = self.create_timer(1 / self.hz, self.cb_main_clock)

        self.sync_frames()

        self.sync_clock = self.create_timer(1 / self.hz, self.cb_sync_clock)

        self.des_pos_f = np.zeros(6)
        self.u_sign = np.zeros(6)

    def sat(self, x):
        return np.clip(x, -1.0, 1.0)

    @property
    def is_running(self) -> bool:
        return self._is_running

    def cb_odom_frame(self) -> None:
        now = Time()
        try:
            t = self.tf_buffer.lookup_transform(self.reference_frame, self.odom_frame, now)
        except Exception as e:
            self.get_logger().error(f"{e}")
            return

        transform = rnp.numpify(t.transform)
        self.pos = transform

    def cb_target_frame(self) -> None:
        now = Time()
        try:
            t = self.tf_buffer.lookup_transform(self.reference_frame, self.target_frame, now)
        except Exception as e:
            self.get_logger().error(f"{e}")
            return

        transform = rnp.numpify(t.transform)
        self.des_pos = transform
        tf_transformations

    def cb_desired_twist(self, msg: TwistStamped):
        self.des_twist = rnp.numpify(msg.twist)

    def cb_odom(self, msg: Odometry):
        self.twist = rnp.numpify(msg.twist.twist)

    def pose_to_vec6(self, pose):
        pos = rnp.numpify(pose.position)
        quat = rnp.numpify(pose.orientation)
        quat = quat[..., (1, 2, 3, 0)]  # xyzw → wxyz
        rpy = tf_transformations.euler_from_quaternion(quat)
        return np.concatenate((pos, rpy), axis=None)

    def T_to_vec6(self, T):
        pos = T[:3, 3]
        rpy = tf_transformations.euler_from_matrix(T[:3, :3])
        return np.concatenate((pos, rpy), axis=None)

    def cb_main_clock(self) -> None:
        if not self._frames_synced:
            return

        dt = 1.0 / self.hz

        # ======================================================
        # 1️⃣ REFERENCE GOVERNOR (softens step input)
        # ======================================================
        des_vec = self.pose_to_vec6(rnp.msgify(Pose, self.des_pos))
        self.des_pos_f += self.alpha * (des_vec - self.des_pos_f) * dt

        # ======================================================
        # 2️⃣ Pose → [x y z roll pitch yaw]
        # ======================================================
        def pose_to_vec(pose):
            pos = rnp.numpify(pose.position)
            quat = rnp.numpify(pose.orientation)
            quat = quat[..., (1, 2, 3, 0)]
            rot = tf_transformations.euler_from_quaternion(quat)
            return np.concatenate((pos, rot), axis=None)

        u_ref = self.des_pos_f
        u_act = self.pose_to_vec6(rnp.msgify(Pose, self.pos))

        # ======================================================
        # 3️⃣ Error + velocity error
        # ======================================================
        e = u_act - u_ref
        e_dot = self.twist - self.des_twist

        # ======================================================
        # 4️⃣ Sliding surface (MFSMC)
        # ======================================================
        # sliding surface
        s = self.A * e + e_dot

        # adaptive gain (6D!)
        k = self.k_min + (self.k_max - self.k_min) * np.minimum(
            1.0, np.abs(e) / self.e_big
        )

        # equivalent control
        u_eq = -k * self.sat(s / self.phi)

        # integral sliding
        self.u_sign += self.sat(s) * dt
        self.u_sign = np.clip(self.u_sign, -1.0, 1.0)

        u_i = self.ki * self.u_sign

        # FINAL CONTROL (6D VECTOR)
        u = self.kd * (u_eq + u_i)

        # ======================================================
        # 9️⃣ Publish wrench (ROV convention)
        # ======================================================
        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = self.get_clock().now().to_msg()
        wrench_msg.header.frame_id = self.odom_frame

        wrench_msg.wrench.force.x = u[0]
        wrench_msg.wrench.force.y = -u[1]
        wrench_msg.wrench.force.z = u[2]

        wrench_msg.wrench.torque.x = -u[5]
        wrench_msg.wrench.torque.y = -u[4]
        wrench_msg.wrench.torque.z = -u[3]

        self.wrench_pub.publish(wrench_msg)

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
    def desired_twist_topic_name(self):
        return self.get_parameter('desired_twist_topic_name').value

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
    def des_twist(self) -> np.ndarray:
        return self.__des_twist

    @des_twist.setter
    def des_twist(self, value: np.ndarray) -> None:
        self.__des_twist = value

    @property
    def pos(self) -> np.ndarray:
        return self.__pos

    @pos.setter
    def pos(self, value: np.ndarray) -> None:
        self.__pos = value

    @property
    def twist(self) -> np.ndarray:
        return self.__twist

    @twist.setter
    def twist(self, value: np.ndarray) -> None:
        self.__twist = value

    @property
    def A(self) -> np.ndarray:
        A_ = self.get_parameter('A').value
        return A_

    # @A.setter
    # def A(self, value: float|np.ndarray) -> None:
    #     self._A = value * np.eye(6)

    @property
    def kd(self) -> np.matrix:
        kd_ = self.get_parameter('kd').value
        return kd_

    # @kd.setter
    # def kd(self, value: float|np.ndarray) -> None:
    #     self._kd = value * np.eye(6)

    @property
    def ki(self) -> np.ndarray:
        ki_ = self.get_parameter('ki').value
        return ki_

    @property
    def alpha(self) -> np.float64:
        return self.get_parameter('alpha').value

    @property
    def phi(self) -> np.ndarray:
        return np.array(self.get_parameter('phi').value)

    @property
    def k_min(self) -> np.ndarray:
        return np.array(self.get_parameter('k_min').value)

    @property
    def k_max(self) -> np.ndarray:
        return np.array(self.get_parameter('k_max').value)

    @property
    def e_big(self) -> np.ndarray:
        return np.array(self.get_parameter('e_big').value)

    # @ki.setter
    # def ki(self, value: float|np.ndarray) -> None:
    #     self._ki = value * np.eye(6)

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
