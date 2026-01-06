from dataclasses import dataclass, field
import math
import os

from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster


@dataclass
class ControlSystemSample:
    time: float
    target_position_x: float
    target_position_y: float
    target_position_z: float
    target_orientation_x: float
    target_orientation_y: float
    target_orientation_z: float
    robot_position_x: float
    robot_position_y: float
    robot_position_z: float
    robot_orientation_x: float
    robot_orientation_y: float
    robot_orientation_z: float
    error_position_x: float = field(init=False)
    error_position_y: float = field(init=False)
    error_position_z: float = field(init=False)
    error_orientation_x: float = field(init=False)
    error_orientation_y: float = field(init=False)
    error_orientation_z: float = field(init=False)

    def __post_init__(self):
        self.error_position_x = self.target_position_x - self.robot_position_x
        self.error_position_y = self.target_position_y - self.robot_position_y
        self.error_position_z = self.target_position_z - self.robot_position_z
        self.error_orientation_x = self.target_orientation_x - self.robot_orientation_x
        self.error_orientation_y = self.target_orientation_y - self.robot_orientation_y
        self.error_orientation_z = self.target_orientation_z - self.robot_orientation_z

    def get_fields(self, delimiter: str = ';'):
        fields = ['time',
                  'target position X',
                  'target position Y',
                  'target position Z',
                  'target orientation X',
                  'target orientation Y',
                  'target orientation Z',
                  'robot position X',
                  'robot position Y',
                  'robot position Z',
                  'robot orientation X',
                  'robot orientation Y',
                  'robot orientation Z',
                  'error position X',
                  'error position Y',
                  'error position Z',
                  'error orientation X',
                  'error orientation Y',
                  'error orientation Z']
        return delimiter.join(fields)

    def get_values(self, delimiter: str = ';'):
        values = [str(round(self.time, 2)),
                  str(round(self.target_position_x, 3)),
                  str(round(self.target_position_y, 3)),
                  str(round(self.target_position_z, 3)),
                  str(round(self.target_orientation_x, 3)),
                  str(round(self.target_orientation_y, 3)),
                  str(round(self.target_orientation_z, 3)),
                  str(round(self.robot_position_x, 3)),
                  str(round(self.robot_position_y, 3)),
                  str(round(self.robot_position_z, 3)),
                  str(round(self.robot_orientation_x, 3)),
                  str(round(self.robot_orientation_y, 3)),
                  str(round(self.robot_orientation_z, 3)),
                  str(round(self.error_position_x, 3)),
                  str(round(self.error_position_y, 3)),
                  str(round(self.error_position_z, 3)),
                  str(round(self.error_orientation_x, 3)),
                  str(round(self.error_orientation_y, 3)),
                  str(round(self.error_orientation_z, 3))]
        return delimiter.join(values)


def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z


class ControlSystemTester(Node):

    def __init__(self):
        Node.__init__(self=self, node_name='control_system_tester')

        self.declare_parameter('frequency', 15)
        self.declare_parameter('listening_time', 60.0)
        self.declare_parameter('output_dir', '/home/dev/ros2_ws/src/control_system/rov_control_system_tester/output')
        self.declare_parameter('output_filename', 'output.txt')

        self.declare_parameter('odom_frame', 'base_link_ref')
        self.declare_parameter('target_frame', 'traj_gen_node')
        self.declare_parameter('reference_frame', 'world_ned')

        self.frequency = self.get_parameter('frequency').value
        self.listening_time = self.get_parameter('listening_time').value
        self.output_dir = self.get_parameter('output_dir').value
        self.output_filename = self.get_parameter('output_filename').value

        self.odom_frame = self.get_parameter('odom_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        self.reference_frame = self.get_parameter('reference_frame').value

        self.tf_buffer = Buffer()
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.dt = 1 / self.frequency

        self.samples: list[ControlSystemSample] = []
        self.number_of_samples = int(self.frequency * self.listening_time)

        self.main_clock = self.create_timer(self.dt, self.cb_main_clock)

    def cb_main_clock(self):
        t_odom = None
        t_ref = None

        try:
            t_odom = self.tf_buffer.lookup_transform(self.odom_frame, self.reference_frame, Time())
            t_ref = self.tf_buffer.lookup_transform(self.target_frame, self.reference_frame, Time())
        except Exception:
            return

        if len(self.samples) == self.number_of_samples:
            self.save()

            self.get_logger().info(f'Test Run Stopped and Saved\n \
                                   \tOutput File: {self.output_filename}')

            self.destroy_timer(self.main_clock)
            return
        elif len(self.samples) == 1:
            self.get_logger().info(f'Starting Control System Benchmark: \
                        \tFrequency: {self.frequency} \
                        \tListening Time: {self.listening_time} \
                        \tOutput Filename: {self.output_filename} \
                        \tOdom Frame: {self.odom_frame} \
                        \tTarget Frame: {self.target_frame} \
                        \tReference Frame: {self.reference_frame}')

        t_ref_rot = euler_from_quaternion(x=t_ref.transform.rotation.x,
                                          y=t_ref.transform.rotation.y,
                                          z=t_ref.transform.rotation.z,
                                          w=t_ref.transform.rotation.w)

        t_odom_rot = euler_from_quaternion(x=t_odom.transform.rotation.x,
                                           y=t_odom.transform.rotation.y,
                                           z=t_odom.transform.rotation.z,
                                           w=t_odom.transform.rotation.w)

        sample = ControlSystemSample(time=len(self.samples) * self.dt,
                                     target_position_x=t_ref.transform.translation.x,
                                     target_position_y=t_ref.transform.translation.y,
                                     target_position_z=t_ref.transform.translation.z,
                                     target_orientation_x=t_ref_rot[0],
                                     target_orientation_y=t_ref_rot[1],
                                     target_orientation_z=t_ref_rot[2],
                                     robot_position_x=t_odom.transform.translation.x,
                                     robot_position_y=t_odom.transform.translation.y,
                                     robot_position_z=t_odom.transform.translation.z,
                                     robot_orientation_x=t_odom_rot[0],
                                     robot_orientation_y=t_odom_rot[1],
                                     robot_orientation_z=t_odom_rot[2])

        self.samples.append(sample)

    def save(self):
        data = self.samples[-1].get_fields() + '\n'

        for sample in self.samples:
            data += sample.get_values() + '\n'

        file_path = os.path.join(self.output_dir, self.output_filename)

        with open(file_path, 'w') as file:
            file.write(data)
