from abc import ABCMeta, ABC, abstractmethod

class Controller(metaclass = ABCMeta):
    def __init__(self, name: str, version: str = None):
        self.controller_name = name
        self.controller_version = version 

    @ABC.abstractmethod
    def run(self) -> bool:
        ...

    @ABC.abstractmethod
    def stop(self) -> bool:
        ...
