# universal_robots_rtde.py

import rtde_control
import rtde_receive
from typing import List
from .robot_interface import RobotInterface

class UniversalRobots_RTDE(RobotInterface):
    def __init__(self, ip: str, port: str):
        
        super().__init__(robot_ip=ip)
        
        self.robot_ip = ip
        self.robot_port = port

        # Load important config params
        self.num_joints = 6
        self.speed_limit = 0.5
        self.acceleration_limit = 0.5

        try:
            self.rtde_c = rtde_control.RTDEControlInterface(self.robot_ip)
            self.rtde_r = rtde_receive.RTDEReceiveInterface(self.robot_ip)
        except Exception as e:
            self.close_connection()
            raise Exception(f"Error initializing RTDE interfaces: {e}")

    def IsConnected(self) -> bool:
        """
        Check if the RTDE interfaces are connected.
        :return: True if both RTDEControlInterface and RTDEReceiveInterface are connected, False otherwise.
        """
        try:
            # Simple check by attempting to get robot version
            version = self.rtde_r.getRobotVersion()
            return True if version else False
        except:
            return False

    def MoveLinearToPosition(self, pose: List[float], speed: float, acceleration: float) -> bool:
        """
        Move the robot linearly to a specified pose.

        :param pose: A list of 6 floats representing the target pose [x, y, z, rx, ry, rz].
        :param speed: The speed at which to move.
        :param acceleration: The acceleration to use.
        :return: True if the movement was successful, False otherwise.
        """

        if not isinstance(pose, list):
            raise ValueError("Pose must be a list.")

        if len(pose) != 6:
            raise ValueError("Pose must have six elements: [x, y, z, rx, ry, rz].")

        speed = speed or self.speed_limit
        acceleration = acceleration or self.acceleration_limit

        try:
            self.rtde_c.moveL(pose, speed, acceleration)
            return True
        except Exception as e:
            print(f"Error moving to position: {e}")
            self.close_connection()
            return False

    def MoveJoint(self, joint_positions: List[float], speed: float, acceleration: float) -> bool:
        """
        Move the robot's joints to specified positions.

        :param joint_positions: A list of floats representing the target joint positions.
        :param speed: The speed at which to move.
        :param acceleration: The acceleration to use.
        :return: True if the movement was successful, False otherwise.
        """

        if not isinstance(joint_positions, list):
            raise ValueError("Joint positions must be a list.")

        if len(joint_positions) != self.num_joints:
            raise ValueError(f"Joint positions must have {self.num_joints} elements.")

        speed = speed or self.speed_limit
        acceleration = acceleration or self.acceleration_limit

        try:
            self.rtde_c.moveJ(joint_positions, speed, acceleration)
            return True
        except Exception as e:
            print(f"Error moving joint positions: {e}")
            self.close_connection()
            return False

    def GetActualQ(self) -> List[float]:
        """
        Get the current joint positions of the robot.

        :return: A list of floats representing the current joint positions.
        """

        try:
            joint_positions = self.rtde_r.getActualQ()
            return joint_positions
        except Exception as e:
            print(f"Error getting actual joint positions: {e}")
            self.close_connection()
            raise

    def GetActualTCPPosition(self) -> List[float]:
        """
        Get the current TCP (Tool Center Point) position of the robot.

        :return: A list of floats representing the current TCP position [x, y, z, rx, ry, rz].
        """

        try:
            pose = self.rtde_r.getActualTCPPose()
            return pose
        except Exception as e:
            print(f"Error getting actual TCP pose: {e}")
            self.close_connection()
            raise

    def stop(self) -> None:
        """
        Stop all robot movements and close RTDE connections.
        """
        self.close_connection()

    def close_connection(self):
        """
        Close the RTDEControlInterface and RTDEReceiveInterface connections.
        """
        try:
            if hasattr(self, 'rtde_c') and self.rtde_c:
                self.rtde_c.stopScript()
                self.rtde_c.disconnect()
                print("RTDEControlInterface disconnected successfully.")
        except Exception as e:
            print(f"Error closing RTDEControlInterface: {e}")

        try:
            if hasattr(self, 'rtde_r') and self.rtde_r:
                self.rtde_r.disconnect()
                print("RTDEReceiveInterface disconnected successfully.")
        except Exception as e:
            print(f"Error closing RTDEReceiveInterface: {e}")
