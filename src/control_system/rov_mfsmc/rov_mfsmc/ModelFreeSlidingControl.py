from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose, Twist, TwistStamped, WrenchStamped, TransformStamped, Transform
import numpy as np
import ros2_numpy as rnp
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster
import tf_transformations
from rov_controll_bus.Controller import Controller


np.printoptions(precision=4, suppress=True)


class ModelFreeSlidingControl(Node, Controller):

    def __init__(self):
        Node.__init__(self=self, node_name='rov_mfsmc_node')
        Controller.__init__(self=self, name='ModelFreeSlidingControl', version='0.1.0')

        self.declare_parameter('desired_position_topic_name', '/target_pose')
        self.declare_parameter('desired_twist_topic_name', '/target_twist')
        self.declare_parameter('odom_topic_name', '/odometry/filtered')

        self.declare_parameter('A', [2.3, 2.3, 2.3, 1.0, 1.0, 0.35])
        self.declare_parameter('kd', [4.5, 4.5, 4.5, 0.5, 1.0, 0.08])
        self.declare_parameter('ki', [0.1, 0.1, 0.1, 0.1, 0.1, 0.4])

        self.declare_parameter('odom_frame', 'base_link')
        self.declare_parameter('target_frame', 'traj_gen_node')
        self.declare_parameter('reference_frame', 'world_ned')
        self.declare_parameter('hz', 120)

        self.desired_twist_sub = self.create_subscription(TwistStamped,
                                                          self.desired_twist_topic_name,
                                                          self.cb_desired_twist, 10)
        self.odom_sub = self.create_subscription(Odometry,
                                                 self.odom_topic_name,
                                                 self.cb_odom, qos_profile=0)

        self.wrench_pub = self.create_publisher(WrenchStamped, 'mfsc_wrench', 0)
        self.sync_trajectory_srv = self.create_service(Trigger, 'sync_trajectory', self.cb_sync_trajectory)

        self.tf_buffer = Buffer()
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.odom_clock = None
        self.target_clock = None
        self.main_clock = None

        self.create_service(Trigger, 'run', self.run)
        self.create_service(Trigger, 'stop', self.stop)

    def run(self, request: Trigger.Request, response: Trigger.Response):
        try:
            self.__des_pos = None
            self.__pos = None
            self.__des_twist = None
            self.__twist = None

            self.u_sign = np.zeros(6)

            self._frames_synced = False

            self.odom_clock = self.create_timer(1 / self.hz, self.cb_odom_frame)
            self.target_clock = self.create_timer(1 / self.hz, self.cb_target_frame)
            self.main_clock = self.create_timer(1 / self.hz, self.cb_main_clock)

            self.sync_frames()

            self.sync_clock = self.create_timer(1 / self.hz, self.cb_sync_clock)

            response.success = True
            response.message = 'ok'

        except Exception:
            response.success = False
            response.message = 'not ok'
        finally:
            return response

    def stop(self, request: Trigger.Request, response: Trigger.Response):
        try:
            self.odom_clock.cancel()
            self.target_clock.cancel()
            self.main_clock.cancel()
            response.success = True
            response.message = 'ok'

        except Exception:
            response.success = False
            response.message = 'not ok'
        finally:
            return response

    def cb_odom_frame(self) -> None:
        now = Time()
        try:
            t = self.tf_buffer.lookup_transform(self.odom_frame, self.reference_frame, now)
        except Exception as e:
            # self.get_logger().error(f'Could not lookup transform: {self.reference_frame} -> {self.odom_frame}')
            self.get_logger().error(f"{e}")
            return

        transform = rnp.numpify(t.transform)
        self.pos = transform

    def cb_target_frame(self) -> None:
        now = Time()
        try:
            t = self.tf_buffer.lookup_transform(self.target_frame, self.reference_frame, now)
        except Exception as e:
            # self.get_logger().error(f'Could not lookup transform: {self.reference_frame} -> {self.target_frame}')
            self.get_logger().error(f"{e}")
            return

        transform = rnp.numpify(t.transform)
        self.des_pos = transform
        tf_transformations

    def cb_desired_twist(self, msg: TwistStamped):
        self.des_twist = rnp.numpify(msg.twist)

    def cb_odom(self, msg: Odometry):
        self.twist = rnp.numpify(msg.twist.twist)

    def cb_main_clock(self) -> None:
        if not self._frames_synced:
            return

        u = np.linalg.inv(self.des_pos) @ self.pos

        u_1 = rnp.msgify(Pose, self.des_pos)
        u_1_pos = rnp.numpify(u_1.position)
        u_1_quat = rnp.numpify(u_1.orientation)
        u_1_quat = u_1_quat[..., (1, 2, 3, 0)]
        u_1_rot = tf_transformations.euler_from_quaternion(u_1_quat)
        u_1 = np.concatenate((u_1_pos, u_1_rot), axis=None)

        u_2 = rnp.msgify(Pose, self.pos)
        u_2_pos = rnp.numpify(u_2.position)
        u_2_quat = rnp.numpify(u_2.orientation)
        u_2_quat = u_2_quat[..., (1, 2, 3, 0)]
        u_2_rot = tf_transformations.euler_from_quaternion(u_2_quat)
        u_2 = np.concatenate((u_2_pos, u_2_rot), axis=None)

        u = u_2 - u_1

        # u: Pose = rnp.msgify(Pose, u)
        # u_pos = rnp.numpify(u.position)
        # u_quat = rnp.numpify(u.orientation)
        # u_quat = u_quat[..., (1, 2, 3, 0)]
        # u_rot = tf_transformations.euler_from_quaternion(u_quat)

        # u = np.concatenate((u_pos, u_rot), axis=None)
        u = np.diag(self.A * u).copy()
        # self.get_logger().info(f"u      : {np.array2string(u, precision=4, floatmode='fixed', suppress_small=True)}")

        vel = self.twist - self.des_twist
        # self.get_logger().info(f"vel    : {np.array2string(vel, precision=4, floatmode='fixed', suppress_small=True)}")

        u += vel
        # self.get_logger().info(f"product: {np.array2string(u, precision=4, floatmode='fixed', suppress_small=True)}")

        self.u_sign += np.sign(u)
        self.u_sign = np.clip(self.u_sign, a_min=-1, a_max=1)

        u_d = np.diag(self.ki * self.u_sign).copy()

        u += u_d
        u = np.diag(self.kd * u)
        # self.get_logger().info(f"wrench : {np.array2string(u, precision=4, floatmode='fixed', suppress_small=True)}")

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
            # self.get_logger().error(msg)
            # self.get_logger().debug(f'{e}')
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
        return A_ * np.eye(6)

    # @A.setter
    # def A(self, value: float|np.ndarray) -> None:
    #     self._A = value * np.eye(6)

    @property
    def kd(self) -> np.matrix:
        kd_ = self.get_parameter('kd').value
        return kd_ * np.eye(6)

    # @kd.setter
    # def kd(self, value: float|np.ndarray) -> None:
    #     self._kd = value * np.eye(6)

    @property
    def ki(self) -> np.ndarray:
        ki_ = self.get_parameter('ki').value
        return ki_ * np.eye(6)

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
