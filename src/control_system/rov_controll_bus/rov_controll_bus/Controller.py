from abc import ABCMeta, abstractmethod


class Controller(metaclass=ABCMeta):
    def __init__(self, name: str, short_name: str = None, version: str = None):
        self.controller_name = name
        self.controller_short_name = short_name
        self.controller_version = version

    @abstractmethod
    def run(self) -> bool:
        ...

    @abstractmethod
    def stop(self) -> bool:
        ...

    @property
    @abstractmethod
    def is_running(self) -> bool:
        ...
