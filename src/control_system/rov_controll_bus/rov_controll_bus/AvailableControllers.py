from dataclasses import dataclass
from rov_controll_bus.Controller import Controller

@dataclass
class AvailableController:
    obj: Controller
    name: str
    short_name: 


class AvailableControllers:
    