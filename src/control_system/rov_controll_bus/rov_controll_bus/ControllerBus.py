import rclpy
from rclpy.node import Node
from rclpy.parameter_service import ParameterNotDeclaredException, ParameterUninitializedException
from std_srvs.srv import Trigger

class ControllerBus(Node):
	def __init__(self) -> None:
		Node.__init__(self=self, node_name='rov_controller_bus', allow_undeclared_parameters=False)
		self._param_prefix = 'c'
		self._param_list = [('version', None), ('init', False)]

		self.parameters = self.load_config()
		self.number_of_controllers = len(self.parameters)

		self.create_service(Trigger, '/controllers/next', self.srv_next)

	def srv_next(self, request: Trigger.Request, response: Trigger.Response) -> None:
		running = self.get_running_controller()
		if not running:
			response.success = False
			response.message = 'None of controller is running'
		
		client = self.create_client(Trigger, f"{running['name']}/stop")
		resp: Trigger.Response = client.call(Trigger.Request())
		if not resp.success:
			response.success = False
			response.message = 'Could not stop previous controller'
			return

		next_controller_index = (running['index'] + 1) % self.number_of_controllers
		next_controller = self.get_controller(next_controller_index)
		client = self.create_client(Trigger, f"{next_controller['name']}/run")
		resp: Trigger.Response = client.call(Trigger.Request())
		response.success = resp.success
		response.message = resp.message

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

			scrapped_parameters[p_name]['is_running'] = False
			scrapped_parameters[p_name]['index'] = p_i

			p_i += 1
		return scrapped_parameters
	
	def get_init_controller(self) -> dict | None:
		for c in self.parameters:
			if c['init']:
				return c
		return None
	
	def get_running_controller(self) -> dict | None:
		for c in self.parameters:
			if c['is_running']:
				return c
		return None
	
	def get_controller(self, index: int) -> dict | None:
		for c in self.parameters:
			if c['index'] == index:
				return c
		return None