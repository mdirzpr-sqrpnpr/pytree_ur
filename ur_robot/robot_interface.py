# robot_interface.py

from abc import ABC, abstractmethod
from typing import List

class RobotInterface(ABC):
    def __init__(self, robot_ip: str):
        self.robot_ip = robot_ip

    @abstractmethod
    def IsConnected(self) -> bool:
        pass

    @abstractmethod
    def MoveLinearToPosition(self, pose: List[float], speed: float, acceleration: float) -> bool:
        pass

    @abstractmethod
    def MoveJoint(self, joint_positions: List[float], speed: float, acceleration: float) -> bool:
        pass

    @abstractmethod
    def GetActualQ(self) -> List[float]:
        pass

    @abstractmethod
    def GetActualTCPPosition(self) -> List[float]:
        pass

    @abstractmethod
    def stop(self) -> None:
        pass
