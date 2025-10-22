import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger, SetBool
from rov_mfsmc.ModelFreeSlidingControl import ModelFreeSlidingControl
from rov_passthrough_control.PassthroughControl import PassthroughControl
from rov_controll_bus.Controller import Controller
from rclpy.callback_groups import ReentrantCallbackGroup


class ControllerBus(Node):
	def __init__(self) -> None:
		Node.__init__(self=self, node_name='rov_controller_bus', allow_undeclared_parameters=False)
		self._param_prefix = 'c'
		self._param_list = [('version', None), ('init', False)]

		self.declare_parameter('init_enabled', True)

		self.init_enabled = self.get_parameter('init_enabled').value

		self.available_controlers: list[Controller] = self.get_available_controlers()

		self.parameters = self.load_config()

		self.number_of_available_controlers = len(self.available_controlers)
		self.number_of_controllers = len(self.parameters)

		self.controller_queue: list[Controller] = self.get_controller_queue()

		self.create_service(Trigger, '/controllers/next', self.srv_next, callback_group=ReentrantCallbackGroup())

		if self.init_enabled:

			if len(self.controller_queue):
				client = self.create_client(Trigger, f'/controllers/{self.controller_queue[0].controller_name}/run')

				while not client.wait_for_service(2.0):
					self.get_logger().info(f"Controller not available: {self.controller_queue[0].controller_name}")

				response = client.call_async(Trigger.Request())
				rclpy.spin_until_future_complete(self, response)

				response = response.result()

				if not response.success:
					raise Exception(f'Could not start controller: {self.controller_queue[0].controller_name}')

			else:
				raise Exception('No controller was found')

	def get_available_controlers(self) -> list[Controller]:
		mfsmc = ModelFreeSlidingControl()
		passthrough_control = PassthroughControl()

		controllers = [mfsmc, passthrough_control]

		return controllers

	def srv_next(self, request: Trigger.Request, response: Trigger.Response) -> None:

		running_controller = self.controller_queue[0]
		next_controller = self.controller_queue[1]

		running_client = self.create_client(Trigger, f'/controllers/{running_controller.controller_name}/stop')

		self.get_logger().warning('Hi -4')

		while not running_client.wait_for_service(2.0):
			self.get_logger().info(f"Controller not available: {running_controller.controller_name}")
		res = running_client.call_async(Trigger.Request())

		self.get_logger().warning('Hi -3')

		rclpy.spin_until_future_complete(self, res, timeout_sec=2)
		self.get_logger().warning('Hi -2')

		res = res.result()

		self.get_logger().warning('Hi -1')


		if not res.success:
			response.success = False
			response.message = 'Could not stop running controller'

		self.get_logger().warning('Hi 1')

		next_client = self.create_client(Trigger, f'/controllers/{next_controller.controller_name}/run')

		self.get_logger().warning('Hi 2')

		while not next_client.wait_for_service(2.0):
			self.get_logger().info(f"Controller not available: {running_controller.controller_name}")

		self.get_logger().warning('Hi 3')

		res = next_client.call_async(Trigger.Request())

		self.get_logger().warning('Hi 4')

		rclpy.spin_until_future_complete(self, res, timeout_sec=2)

		self.get_logger().warning('Hi 5')

		res = res.result()

		self.get_logger().warning('Hi 6')

		if not res.success:
			response.success = False
			response.message = 'Could not start controller'

		self.get_logger().warning('Hi 7')

		self.controller_queue.pop(0)
		self.controller_queue.append(running_controller)

		response.success = True
		response.message = 'ok'

		return response

	def srv_enable(self, request: SetBool.Request, response: SetBool.Response) -> None:

		if not len(self.controller_queue):
			response.success = False
			response.message = 'There is no controllers in the queue'

		if request.data and not self.controller_queue[0].is_running:
			client = self.create_client(Trigger, f'/controllers/{self.controller_queue[0].controller_name}/run')
			resp: Trigger.Response = client.call(Trigger.Request())

			if not resp.success:
				response.success = False
				response.message = f'Could not start controller {self.controller_queue[0].controller_name}'

			self.destroy_client(client)

			response.success = True
			response.message = 'ok'
		elif request.data and self.controller_queue[0].is_running:
			response.success = True
			response.message = 'ok'
		elif not request.data and self.controller_queue[0].is_running:
			client = self.create_client(Trigger, f'/controllers/{self.controller_queue[0].controller_name}/stop')
			resp: Trigger.Response = client.call()

			if not resp.success:
				response.success = False
				response.message = f'Could not stop controller {self.controller_queue[0].controller_name}'

			self.destroy_client(client)

			response.success = True
			response.message = 'ok'
		else:

			response.success = True
			response.message = 'ok'

		return response

	def load_config(self) -> dict:
		scrapped_parameters = dict()

		more_params = True
		p_i = 0
		while more_params:
			p_name = f'{self._param_prefix}{p_i}'
			self.declare_parameter(p_name, None)

			name = self.get_parameter(p_name).value
			if not isinstance(name, str):
				more_params = False
				continue

			scrapped_parameters[p_name] = dict()
			scrapped_parameters[p_name]['name'] = name

			for param, default in self._param_list:
				p_sub_name = f'{self._param_prefix}{p_i}_{param}'

				self.declare_parameter(p_sub_name, None)

				p = self.get_parameter(p_sub_name).value

				if not (isinstance(p, str) or isinstance(p, bool)):
					p = default

				scrapped_parameters[p_name][param] = p

			scrapped_parameters[p_name]['index'] = p_i

			p_i += 1
		return scrapped_parameters

	def get_init_controller(self) -> dict | None:
		for c in self.parameters:
			if c['init']:
				return c
		return None

	def get_controller(self, index: int) -> dict | None:
		for c in self.parameters:
			if c['index'] == index:
				return c
		return None

	def get_controller_queue(self) -> list:
		queue = []

		self.get_logger().warning(f"{self.parameters}")

		for param in self.parameters:
			for c in self.available_controlers:
				self.get_logger().warning(f"{c}\n{param}")
				if not (c.controller_name == self.parameters[param]['name'] or c.controller_short_name == self.parameters[param]['name']):
					continue

				if self.parameters[param]['init']:
					queue.insert(0, c)
				else:
					queue.append(c)

		return queue
