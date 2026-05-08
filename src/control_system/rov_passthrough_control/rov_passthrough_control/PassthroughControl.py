from rclpy.node import Node
from std_srvs.srv import Trigger

from ds4_driver_msgs.msg import Status
from geometry_msgs.msg import Wrench, WrenchStamped
from sensor_msgs.msg import Joy

import numpy as np


class PassthroughControl(Node):
    """

    Class managing different controllers for creating wrenches.

    Useful for testing and applying forces to robot

    :param controller: Controller type. Options are:\n
        - **ds4**: DUALSHOCK 4 (default)
        - **joy**: Regular joystick (not implemented)
        - **station**: Control station (not implemented)
    :type controller: str

    :param wrench_topic_name: Name of the published Wrench topic.
        Default: "joy_wrench"
    :type wrench_topic_name: str

    :param wrench_stamped_topic_name: Name of the published WrenchStamped topic.
        Default: "joy_wrench_stmp"
    :type wrench_stamped_topic_name: str

    :param send_stamped: Whether to create a WrenchStamped topic.
        Default: True
    :type send_stamped: bool

    :param frame_id: Frame ID to use in stamped messages.
        Default: "base_link"
    :type frame_id: str

    :param max_norm: Maximum value of the normalized wrench.
        Default: 1.0
    :type max_norm: float

    :param equalization_type: Type of equalization used in the normalization function. Options are:\n
        - **LINEAR**
        - **SQUARE**
        - **CUBE**
        - **INV_SQUARE**
        - **INV_CUBE**\n
        Default: "LINEAR"
    :type equalization_type: str

    :param ds4_force_x:
        Default: "axis_left_y"
    :type ds4_force_x: str

    :param ds4_force_y:
        Default: "axis_left_x"
    :type ds4_force_y: str

    :param ds4_force_z:
        Default: "axis_right_y"
    :type ds4_force_z: str

    :param ds4_torque_x:
        Default: "axis_right_y"
    :type ds4_torque_x: str

    :param ds4_torque_y:
    :type ds4_torque_y: str

    :param ds4_torque_z:
    :type ds4_torque_z: str
    """

    _is_running = False

    def __init__(self):
        """
        Initializes the joystick-to-wrench conversion node.

        This constructor sets up ROS parameters, publishers, and subscriptions for processing joystick
        input and publishing normalized wrench messages.

        :param Node node: The ROS 2 node instance used for logging, parameter management, and communication.

        ROS Parameters:
            - **controller** (:obj:`str`, default: `'ds4'`): Specifies the controller type. Options:
                - `'ds4'`: DUALSHOCK 4 (default)
                - `'joy'`: Regular joystick (not implemented)
                - `'station'`: Control station (not implemented)
            - **wrench_topic_name** (:obj:`str`, default: `'joy_wrench'`): Name of the published Wrench topic.
            - **wrench_stamped_topic_name** (:obj:`str`, default: `'joy_wrench_stmp'`): Name of the published WrenchStamped
            topic.
            - **send_stamped** (:obj:`bool`, default: `True`): Determines whether to publish a `WrenchStamped` message.
            - **frame_id** (:obj:`str`, default: `'base_link'`): Frame ID for the header of stamped messages.
            - **max_norm** (:obj:`float`, default: `1.0`): Maximum norm value for the normalized wrench.
            - **equalization_type** (:obj:`str`, default: `'linear'`): Type of equalization used in normalization. Options:
                - `'LINEAR'` (default)
                - `'SQUARE'`
                - `'CUBE'`
                - `'INVERSE_SQUARE'`
                - `'INVERSE_CUBE'`
            - **ds4_force_x** (:obj:`str`, default: `'axis_left_y'`): Joystick axis for force in the X direction.
            - **ds4_force_y** (:obj:`str`, default: `'axis_left_x'`): Joystick axis for force in the Y direction.
            - **ds4_force_z** (:obj:`str`, default: `'axis_right_y'`): Joystick axis for force in the Z direction.
            - **ds4_torque_x** (:obj:`str`, default: `'axis_right_x'`): Joystick axis for torque around the X-axis.
            - **ds4_torque_y** (:obj:`str`, default: `''`): Joystick axis for torque around the Y-axis (not assigned).
            - **ds4_torque_z** (:obj:`str`, default: `''`): Joystick axis for torque around the Z-axis (not assigned).

        Attributes:
            - **controller** (:obj:`str`): Uppercase version of the controller type.
            - **wrench_topic_name** (:obj:`str`): Name of the Wrench topic.
            - **wrench_stamped_topic_name** (:obj:`str`): Name of the WrenchStamped topic.
            - **send_stamped** (:obj:`bool`): Whether to send stamped messages.
            - **frame_id** (:obj:`str`): Frame ID for stamped messages.
            - **max_norm** (:obj:`float`): Maximum norm for normalization.
            - **equalization_type** (:obj:`str`): Uppercase version of the equalization type.
            - **pub_joy_wrench** (:obj:`Publisher`): Publisher for Wrench or WrenchStamped messages.
            - **sub_ds4_driver** (:obj:`Subscription`): Subscription to the joystick input topic (if using DS4 controller).
            - **joy_force_x, joy_force_y, joy_force_z** (:obj:`str`): Joystick axes for force input.
            - **joy_torque_x, joy_torque_y, joy_torque_z** (:obj:`str`): Joystick axes for torque input.

        Raises:
            - None

        Returns:
            None
        """
        Node.__init__(self=self, node_name='rov_passthrough_control_node')

        self._logger = self.get_logger()

        # Declare parameters

        # Specifies the type of controller being used.
        # Options:
        # - 'ds4': DUALSHOCK 4 controller (default).
        # - 'joy': Regular joystick (not implemented).
        # - 'station': Control station (not implemented).
        self.declare_parameter("controller", 'ds4')

        # The name of the ROS topic where a Wrench message will be published if send_stamped is False.
        self.declare_parameter("wrench_topic_name", "joy_wrench")

        # The name of the ROS topic where a WrenchStamped message will be published if send_stamped is True.
        self.declare_parameter("wrench_stamped_topic_name", "calculated_wrench")

        # Indicates whether the published message should include a timestamp (WrenchStamped) or not (Wrench).
        self.declare_parameter("send_stamped", True)

        # The frame ID to use in the header of the WrenchStamped messages.
        self.declare_parameter("frame_id", "base_link")

        # The maximum allowed value for the normalized wrench vector's magnitude.
        self.declare_parameter("max_norm", 20000.0)

        # The maximum allowed value for the normalized force in wrench vector's magnitude.
        self.declare_parameter("max_force", 10.0)

        # The maximum allowed value for the normalized torque in wrench vector's magnitude.
        self.declare_parameter("max_torque", 1.0)

        # Specifies the equalization method to be used during normalization.
        # Options:
        # - 'LINEAR'
        # - 'SQUARE'
        # - 'CUBE'
        # - 'INVERSE_SQUARE'
        # - 'INVERSE_CUBE'
        self.declare_parameter("equalization_type", "INVERSE_SQUARE")

        # Axis mapping for joystick input to control forces (x, y, z) using the DUALSHOCK 4 controller.
        self.declare_parameter("ds4_force_x", "axis_left_y")  # Force along x-axis.
        self.declare_parameter("ds4_force_y", "axis_left_x")  # Force along y-axis.
        self.declare_parameter("ds4_force_z", "axis_right_y")  # Force along z-axis.

        # Axis mapping for joystick input to control torques (x, y, z) using the DUALSHOCK 4 controller.
        self.declare_parameter("ds4_torque_x", "axis_right_x")  # Torque about x-axis.
        self.declare_parameter("ds4_torque_y", "")  # Torque about y-axis (not mapped).
        self.declare_parameter("ds4_torque_z", "")  # Torque about z-axis (not mapped).

        # Axis index mapping for regular joystick (sensor_msgs/Joy) input to control forces (x, y, z).
        self.declare_parameter("joy_axis_force_x", 1)  # Left stick vertical (forward/back)
        self.declare_parameter("joy_axis_force_y", 0)  # Left stick horizontal (left/right)
        self.declare_parameter("joy_axis_force_z", 4)  # Right stick vertical (up/down)

        # Axis index mapping for regular joystick (sensor_msgs/Joy) input to control torques (x, y, z).
        self.declare_parameter("joy_axis_torque_x", 3)  # Right stick horizontal (roll)
        self.declare_parameter("joy_axis_torque_y", -1)  # Torque about y-axis (not mapped by default)
        self.declare_parameter("joy_axis_torque_z", -1)  # Torque about z-axis (not mapped by default)

        # Button index mapping for regular joystick (sensor_msgs/Joy) input for additional torque axes.
        self.declare_parameter("joy_button_torque_y_pos", -1)  # Button for positive torque about y-axis
        self.declare_parameter("joy_button_torque_y_neg", -1)  # Button for negative torque about y-axis
        self.declare_parameter("joy_button_torque_z_pos", 5)  # Button for positive torque about z-axis (e.g. R1)
        self.declare_parameter("joy_button_torque_z_neg", 4)  # Button for negative torque about z-axis (e.g. L1)

        # Topic name for joy messages
        self.declare_parameter("joy_topic_name", "joy")

        # Whether to inverse input in calculated wrench
        self.declare_parameter('inv_force_x', False)
        self.declare_parameter('inv_force_y', False)
        self.declare_parameter('inv_force_z', False)

        self.declare_parameter('inv_torque_x', False)
        self.declare_parameter('inv_torque_y', False)
        self.declare_parameter('inv_torque_z', False)

        self.sub_ds4_driver = None
        self.sub_joy = None
        self.pub_joy_wrench = None

        # Get parameters
        self.controller = self.get_parameter("controller").value
        self.controller = str(self.controller).upper()

        self.wrench_topic_name = self.get_parameter("wrench_topic_name").value
        self.wrench_stamped_topic_name = self.get_parameter("wrench_stamped_topic_name").value

        self.send_stamped = self.get_parameter("send_stamped").value
        self.frame_id = self.get_parameter("frame_id").value

        self.max_norm = self.get_parameter('max_norm').value

        self.max_force = self.get_parameter('max_force').value
        self.max_torque = self.get_parameter('max_torque').value

        self.equalization_type = self.get_parameter("equalization_type").value
        self.equalization_type = str(self.equalization_type).upper()

        self.inv_force_x = self.get_parameter('inv_force_x').value
        self.inv_force_y = self.get_parameter('inv_force_y').value
        self.inv_force_z = self.get_parameter('inv_force_z').value

        self.inv_torque_x = self.get_parameter('inv_torque_x').value
        self.inv_torque_y = self.get_parameter('inv_torque_y').value
        self.inv_torque_z = self.get_parameter('inv_torque_z').value

        # Create publishers
        if self.send_stamped:
            # If published message is WrenchStamped

            self.pub_joy_wrench = self.create_publisher(WrenchStamped, self.wrench_stamped_topic_name, 0)
        else:
            # if published message is Wrench

            self.pub_joy_wrench = self.create_publisher(Wrench, self.wrench_topic_name, 0)

        if self.controller == 'DS4':
            # Subscribe to ds4 topics

            self.sub_ds4_driver = self.create_subscription(Status, "status", self.cb_ds4_driver, 0)

            # Get name of axis on which wrench will be calculated
            self.joy_force_x = self.get_parameter("ds4_force_x").value  # Force along x-axis.
            self.joy_force_y = self.get_parameter("ds4_force_y").value  # Force along y-axis.
            self.joy_force_z = self.get_parameter("ds4_force_z").value  # Force along z-axis.

            self.joy_torque_x = self.get_parameter("ds4_torque_x").value  # Torque along x-axis.
            self.joy_torque_y = self.get_parameter("ds4_torque_y").value  # Torque along x-axis. (not mapped by default)
            self.joy_torque_z = self.get_parameter("ds4_torque_z").value  # Torque along x-axis. (not mapped by default)

        elif self.controller == 'JOY':
            joy_topic_name = self.get_parameter("joy_topic_name").value

            self.sub_joy = self.create_subscription(Joy, 'joy', self.cb_joy, 0)

            self.joy_axis_force_x = self.get_parameter("joy_axis_force_x").value
            self.joy_axis_force_y = self.get_parameter("joy_axis_force_y").value
            self.joy_axis_force_z = self.get_parameter("joy_axis_force_z").value

            self.joy_axis_torque_x = self.get_parameter("joy_axis_torque_x").value
            self.joy_axis_torque_y = self.get_parameter("joy_axis_torque_y").value
            self.joy_axis_torque_z = self.get_parameter("joy_axis_torque_z").value

            self.joy_button_torque_y_pos = self.get_parameter("joy_button_torque_y_pos").value
            self.joy_button_torque_y_neg = self.get_parameter("joy_button_torque_y_neg").value
            self.joy_button_torque_z_pos = self.get_parameter("joy_button_torque_z_pos").value
            self.joy_button_torque_z_neg = self.get_parameter("joy_button_torque_z_neg").value

            self._logger.info(f"JOY controller initialized, subscribing to '{joy_topic_name}'")

        elif self.controller == 'STATION':
            # TODO: implement 'station' controller

            self._logger.fatal(f"Controller: {self.controller} not implemented yet")

        else:
            # Wrong controller parameter
            self._logger.error(f"Controller type: {self.controller} is not supported")
            self._logger.error("Use: DS4, JOY or STATION instead")

    def run(self, request: Trigger.Request, response: Trigger.Response):
        # Get parameters
        self.controller = self.get_parameter("controller").value
        self.controller = str(self.controller).upper()

        self.wrench_topic_name = self.get_parameter("wrench_topic_name").value
        self.wrench_stamped_topic_name = self.get_parameter("wrench_stamped_topic_name").value

        self.send_stamped = self.get_parameter("send_stamped").value
        self.frame_id = self.get_parameter("frame_id").value

        self.max_norm = self.get_parameter('max_norm').value

        self.max_force = self.get_parameter('max_force').value
        self.max_torque = self.get_parameter('max_torque').value

        self.equalization_type = self.get_parameter("equalization_type").value
        self.equalization_type = str(self.equalization_type).upper()

        self.inv_force_x = self.get_parameter('inv_force_x').value
        self.inv_force_y = self.get_parameter('inv_force_y').value
        self.inv_force_z = self.get_parameter('inv_force_z').value

        self.inv_torque_x = self.get_parameter('inv_torque_x').value
        self.inv_torque_y = self.get_parameter('inv_torque_y').value
        self.inv_torque_z = self.get_parameter('inv_torque_z').value

        # Create publishers
        if self.send_stamped:
            # If published message is WrenchStamped

            self.pub_joy_wrench = self.create_publisher(WrenchStamped, self.wrench_stamped_topic_name, 0)
        else:
            # if published message is Wrench

            self.pub_joy_wrench = self.create_publisher(Wrench, self.wrench_topic_name, 0)

        if self.controller == 'DS4':
            # Subscribe to ds4 topics

            self.sub_ds4_driver = self.create_subscription(Status, "status", self.cb_ds4_driver, 0)

            # Get name of axis on which wrench will be calculated
            self.joy_force_x = self.get_parameter("ds4_force_x").value  # Force along x-axis.
            self.joy_force_y = self.get_parameter("ds4_force_y").value  # Force along y-axis.
            self.joy_force_z = self.get_parameter("ds4_force_z").value  # Force along z-axis.

            self.joy_torque_x = self.get_parameter("ds4_torque_x").value  # Torque along x-axis.
            self.joy_torque_y = self.get_parameter("ds4_torque_y").value  # Torque along x-axis. (not mapped by default)
            self.joy_torque_z = self.get_parameter("ds4_torque_z").value  # Torque along x-axis. (not mapped by default)

        elif self.controller == 'JOY':
            joy_topic_name = self.get_parameter("joy_topic_name").value

            self.sub_joy = self.create_subscription(Joy, joy_topic_name, self.cb_joy, 0)

            self.joy_axis_force_x = self.get_parameter("joy_axis_force_x").value
            self.joy_axis_force_y = self.get_parameter("joy_axis_force_y").value
            self.joy_axis_force_z = self.get_parameter("joy_axis_force_z").value

            self.joy_axis_torque_x = self.get_parameter("joy_axis_torque_x").value
            self.joy_axis_torque_y = self.get_parameter("joy_axis_torque_y").value
            self.joy_axis_torque_z = self.get_parameter("joy_axis_torque_z").value

            self.joy_button_torque_y_pos = self.get_parameter("joy_button_torque_y_pos").value
            self.joy_button_torque_y_neg = self.get_parameter("joy_button_torque_y_neg").value
            self.joy_button_torque_z_pos = self.get_parameter("joy_button_torque_z_pos").value
            self.joy_button_torque_z_neg = self.get_parameter("joy_button_torque_z_neg").value

            self._logger.info(f"JOY controller initialized, subscribing to '{joy_topic_name}'")

        elif self.controller == 'STATION':
            # TODO: implement 'station' controller

            self._logger.fatal(f"Controller: {self.controller} not implemented yet")

        else:
            # Wrong controller parameter
            self._logger.error(f"Controller type: {self.controller} is not supported")
            self._logger.error("Use: DS4, JOY or STATION instead")

        self._is_running = True
        response.success = True
        response.message = 'ok'
        self.get_logger().info(f"started with status: {self.is_running}")

        return response

    def stop(self, request: Trigger.Request, response: Trigger.Response):
        if self.sub_ds4_driver is not None:
            self.destroy_subscription(self.sub_ds4_driver)
        if self.sub_joy is not None:
            self.destroy_subscription(self.sub_joy)
        self.destroy_publisher(self.pub_joy_wrench)
        self._is_running = False
        response.success = True
        response.message = 'ok'

        self.get_logger().info(f"Controller {self.controller} stopped with status: {response.success}")
        return response

    @property
    def is_running(self) -> bool:
        return self._is_running

    def _get_joy_axis(self, axes, index):
        """Safely get axis value from Joy message by index. Returns 0.0 if index is invalid."""
        if index < 0 or index >= len(axes):
            return 0.0
        return float(axes[index])

    def _get_joy_button(self, buttons, index):
        """Safely get button value from Joy message by index. Returns 0.0 if index is invalid."""
        if index < 0 or index >= len(buttons):
            return 0.0
        return float(buttons[index])

    def _publish_wrench(self, wrench_values):
        """
        Publishes a wrench message (Wrench or WrenchStamped) with the given raw values,
        after normalization.

        :param wrench_values: Tuple of (fx, fy, fz, tx, ty, tz) raw values.
        """
        now = self.get_clock().now()

        if self.send_stamped:
            wrench_msg = WrenchStamped()
            wrench_msg.header.frame_id = self.frame_id
            wrench_msg.header.stamp = now.to_msg()
            w = wrench_msg.wrench
        else:
            wrench_msg = Wrench()
            w = wrench_msg

        w.force.x, w.force.y, w.force.z = wrench_values[:3]
        w.torque.x, w.torque.y, w.torque.z = wrench_values[3:]

        w = self.__normalize_joy_input(w)

        if self.send_stamped:
            wrench_msg.wrench.force.x = w.force.x
            wrench_msg.wrench.force.y = w.force.y
            wrench_msg.wrench.force.z = w.force.z
            wrench_msg.wrench.torque.x = w.torque.x
            wrench_msg.wrench.torque.y = w.torque.y
            wrench_msg.wrench.torque.z = w.torque.z
        else:
            wrench_msg.force.x = w.force.x
            wrench_msg.force.y = w.force.y
            wrench_msg.force.z = w.force.z
            wrench_msg.torque.x = w.torque.x
            wrench_msg.torque.y = w.torque.y
            wrench_msg.torque.z = w.torque.z

        self.pub_joy_wrench.publish(wrench_msg)

    def cb_joy(self, msg: Joy):
        """
        Callback function for processing joystick input from the `sensor_msgs/Joy` topic.

        This function converts joystick axes and buttons into a `Wrench` or `WrenchStamped` message,
        normalizes the input, and publishes the resulting message to the configured wrench topic.

        :param Joy msg: The Joy message containing axes and buttons arrays.
        """
        try:
            fx = self._get_joy_axis(msg.axes, self.joy_axis_force_x)
            fy = self._get_joy_axis(msg.axes, self.joy_axis_force_y)
            fz = self._get_joy_axis(msg.axes, self.joy_axis_force_z)

            tx = self._get_joy_axis(msg.axes, self.joy_axis_torque_x)

            if self.joy_axis_torque_y >= 0:
                ty = self._get_joy_axis(msg.axes, self.joy_axis_torque_y)
            else:
                ty = self._get_joy_button(msg.buttons, self.joy_button_torque_y_pos) \
                    - self._get_joy_button(msg.buttons, self.joy_button_torque_y_neg)

            if self.joy_axis_torque_z >= 0:
                tz = self._get_joy_axis(msg.axes, self.joy_axis_torque_z)
            else:
                tz = self._get_joy_button(msg.buttons, self.joy_button_torque_z_pos) \
                    - self._get_joy_button(msg.buttons, self.joy_button_torque_z_neg)

        except Exception as e:
            self._logger.error(f"Error reading joy input: {e}")
            return

        self._publish_wrench((fx, fy, fz, tx, ty, tz))

    def cb_ds4_driver(self, msg: Status):
        """
        Callback function for processing joystick input from the `ds4_driver` topic.

        This function converts joystick input into a `Wrench` or `WrenchStamped` message, 
        normalizes the input, and publishes the resulting message to the configured wrench topic.

        :param Status msg: The message received from the `ds4_driver` topic containing joystick input.

        ROS Parameters:
            - **send_stamped** (:obj:`bool`): If `True`, publishes a `WrenchStamped` message; otherwise, publishes a `Wrench`
            message.
            - **frame_id** (:obj:`str`): Frame ID to use in the header of the stamped message.
            - **joy_force_x, joy_force_y, joy_force_z** (:obj:`str`): Names of the joystick input attributes for force in
            the X, Y, and Z directions.
            - **joy_torque_x** (:obj:`str`): Name of the joystick input attribute for torque around the X-axis.

        Attributes:
            - **wrench_msg** (:class:`Wrench` or :class:`WrenchStamped`): The wrench message to be published.
            - **w** (:class:`Wrench`): The normalized wrench.

        Exceptions:
            - **Exception**: Logs an error if joystick input cannot be processed correctly.

        Returns:
            None
        """

        now = self.get_clock().now()

        if self.send_stamped:
            wrench_msg = WrenchStamped()
            wrench_msg.header.frame_id = self.frame_id
            wrench_msg.header.stamp = now.to_msg()
            w = wrench_msg.wrench
        else:
            wrench_msg = Wrench()
            w = wrench_msg

        try:
            # Calculate raw force and torque from joystick input
            w.force.x = float(getattr(msg, self.joy_force_x))
            w.force.y = float(getattr(msg, self.joy_force_y))
            w.force.z = float(getattr(msg, self.joy_force_z))

            w.torque.x = float(msg.button_dpad_right - msg.button_dpad_left)

            if self.joy_torque_y is None or self.joy_torque_y == "":
                # Assign default values if joy_torque_y is not set

                w.torque.y = float(msg.axis_r2 - msg.axis_l2)

            else:
                w.torque.y = float(msg.axis_r2 - msg.axis_l2)

            if self.joy_torque_z is None or self.joy_torque_y == "":
                # Assign default values if joy_torque_z is not set

                w.torque.z = float(msg.axis_right_x)
            else:

                w.torque.z = float(msg.axis_right_x)

        except Exception as e:
            self._logger.error(f"Wrong wrench: {e}, {w}")

        w = self.__normalize_joy_input(w)

        # Assign normalized values back to the wrench message
        if self.send_stamped:
            wrench_msg.wrench.force.x = w.force.x
            wrench_msg.wrench.force.y = w.force.y
            wrench_msg.wrench.force.z = w.force.z

            wrench_msg.wrench.torque.x = w.torque.x
            wrench_msg.wrench.torque.y = w.torque.y
            wrench_msg.wrench.torque.z = w.torque.z
        else:
            wrench_msg.force.x = w.force.x
            wrench_msg.force.y = w.force.y
            wrench_msg.force.z = w.force.z

            wrench_msg.torque.x = w.torque.x
            wrench_msg.torque.y = w.torque.y
            wrench_msg.torque.z = w.torque.z

        # Publish the normalized wrench message
        self.pub_joy_wrench.publish(wrench_msg)

    def __normalize_joy_input(self, w):
        """
        Normalizes joystick input using the selected equalization type.

        Supported equalization types:
            - **SQUARE**
            - **CUBE**
            - **INVERSE SQUARE**
            - **INVERSE CUBE**
            - **LINEAR** (default)

        :param Wrench w: The input wrench message to be normalized.
        :return: The normalized wrench message.
        :rtype: Wrench
        """
        wrench_vector = np.array([
            w.force.x, w.force.y, w.force.z,
            w.torque.x, w.torque.y, w.torque.z,
        ])
        # Calculate the norm
        norm = np.linalg.norm(wrench_vector)

        magnitude_clamped = np.clip(norm, 0, 1)

        if norm <= 0:
            return w

        eq_type = self.equalization_type

        if eq_type == "SQUARE":
            wrench_vector = (wrench_vector / norm) * np.power(magnitude_clamped, 2)

        elif eq_type == "CUBE":
            wrench_vector = (wrench_vector / norm) * np.power(magnitude_clamped, 3)

        elif eq_type == "INV_SQUARE":
            wrench_vector = (wrench_vector / norm) * (1 - pow(magnitude_clamped - 1, 2))

        elif eq_type == "INV_CUBE":
            wrench_vector = (wrench_vector / norm) * (1 + pow(magnitude_clamped - 1, 3))

        else:
            wrench_vector = (wrench_vector / norm) * magnitude_clamped

        # Update the wrench components with normalized values
        w.force.x, w.force.y, w.force.z = wrench_vector[:3] * self.max_force
        w.torque.x, w.torque.y, w.torque.z = wrench_vector[3:] * self.max_torque

        w.force.x = -w.force.x if self.inv_force_x else w.force.x
        w.force.y = -w.force.y if self.inv_force_y else w.force.y
        w.force.z = -w.force.z if self.inv_force_z else w.force.z

        w.torque.x = -w.torque.x if self.inv_torque_x else w.torque.x
        w.torque.y = -w.torque.y if self.inv_torque_y else w.torque.y
        w.torque.z = -w.torque.z if self.inv_torque_z else w.torque.z

        return w
