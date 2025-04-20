import time
import py_trees

from bt_py.bt_nodes import (
    CheckConnection, MoveJoint, MoveLinear,
    GetJoint, GetTCP, StopRobot, TargetReached
)

# Simulation helper
def simulate_action(action: str, duration_sec: float):
    print(f"Simulating: {action}...")
    time.sleep(duration_sec)
    print(f"{action} completed!")

# Simulation subclasses override update() to fake execution
class CheckConnectionSim(CheckConnection):
    def update(self):
        simulate_action("CheckConnection", 0.5)
        self.blackboard.robot_connected = True
        return py_trees.common.Status.SUCCESS

class MoveLinearSim(MoveLinear):
    def update(self):
        simulate_action("MoveLinear", 1.0)
        self.blackboard.last_move = {"type": "linear", "pose": self.pose}
        return py_trees.common.Status.SUCCESS

class MoveJointSim(MoveJoint):
    def update(self):
        simulate_action("MoveJoint", 1.0)
        self.blackboard.last_move = {"type": "joint", "joints": self.joints}
        return py_trees.common.Status.SUCCESS

class GetJointSim(GetJoint):
    def update(self):
        simulate_action("GetJoint", 0.2)
        self.blackboard.current_joints = [0.0] * 6
        return py_trees.common.Status.SUCCESS

class GetTCPSim(GetTCP):
    def update(self):
        simulate_action("GetTCP", 0.2)
        self.blackboard.current_tcp = [0.0] * 6
        return py_trees.common.Status.SUCCESS

class StopRobotSim(StopRobot):
    def update(self):
        simulate_action("StopRobot", 0.1)
        self.blackboard.robot_stopped = True
        return py_trees.common.Status.SUCCESS

class TargetReachedSim(TargetReached):
    def update(self):
        simulate_action("TargetReached", 0.1)
        self.blackboard.target_reached = True
        return py_trees.common.Status.SUCCESS