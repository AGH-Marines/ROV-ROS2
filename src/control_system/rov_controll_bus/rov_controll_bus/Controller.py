from abc import ABCMeta, abstractmethod


class Controller(metaclass=ABCMeta):
    def __init__(self, name: str, version: str = None):
        self.controller_name = name
        self.controller_version = version 

    @abstractmethod
    def run(self) -> bool:
        ...

    @abstractmethod
    def stop(self) -> bool:
        ...
