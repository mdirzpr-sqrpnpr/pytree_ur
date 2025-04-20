import py_trees
import py_trees.blackboard as bt_bb
from ur_robot.ur_rtde import UniversalRobots_RTDE


class CheckConnection(py_trees.behaviour.Behaviour):
    def __init__(self, robot: UniversalRobots_RTDE, name: str = "CheckConnection"):
        super().__init__(name)
        self.robot = robot
        self.blackboard = bt_bb.Blackboard()

    def initialise(self):
        pass

    def update(self):
        status = self.robot.IsConnected()
        self.blackboard.robot_connected = status
        return py_trees.common.Status.SUCCESS if status else py_trees.common.Status.FAILURE

class MoveLinear(py_trees.behaviour.Behaviour):
    def __init__(self,
                 robot: UniversalRobots_RTDE,
                 pose: list,
                 speed: float = None,
                 accel: float = None,
                 name: str = "MoveLinear"):
        super().__init__(name)
        self.robot = robot
        self.pose = pose
        self.speed = speed
        self.accel = accel
        self.blackboard = bt_bb.Blackboard()
        self.executed = False
        self.result = False

    def initialise(self):
        self.executed = False

    def update(self):
        if not self.executed:
            self.result = self.robot.MoveLinearToPosition(self.pose, self.speed, self.accel)
            self.executed = True
        self.blackboard.last_move = {"type": "linear", "pose": self.pose}
        return py_trees.common.Status.SUCCESS if self.result else py_trees.common.Status.FAILURE

class MoveJoint(py_trees.behaviour.Behaviour):
    def __init__(self,
                 robot: UniversalRobots_RTDE,
                 joints: list,
                 speed: float = None,
                 accel: float = None,
                 name: str = "MoveJoint"):
        super().__init__(name)
        self.robot = robot
        self.joints = joints
        self.speed = speed
        self.accel = accel
        self.blackboard = bt_bb.Blackboard()
        self.executed = False
        self.result = False

    def initialise(self):
        self.executed = False

    def update(self):
        if not self.executed:
            self.result = self.robot.MoveJoint(self.joints, self.speed, self.accel)
            self.executed = True
        self.blackboard.last_move = {"type": "joint", "joints": self.joints}
        return py_trees.common.Status.SUCCESS if self.result else py_trees.common.Status.FAILURE

class GetJoint(py_trees.behaviour.Behaviour):
    def __init__(self, robot: UniversalRobots_RTDE, name: str = "GetJoint"):
        super().__init__(name)
        self.robot = robot
        self.blackboard = bt_bb.Blackboard()

    def update(self):
        joints = self.robot.GetActualQ()
        self.blackboard.current_joints = joints
        return py_trees.common.Status.SUCCESS

class GetTCP(py_trees.behaviour.Behaviour):
    def __init__(self, robot: UniversalRobots_RTDE, name: str = "GetTCP"):
        super().__init__(name)
        self.robot = robot
        self.blackboard = bt_bb.Blackboard()

    def update(self):
        tcp = self.robot.GetActualTCPPosition()
        self.blackboard.current_tcp = tcp
        return py_trees.common.Status.SUCCESS

class StopRobot(py_trees.behaviour.Behaviour):
    def __init__(self, robot: UniversalRobots_RTDE, name: str = "StopRobot"):
        super().__init__(name)
        self.robot = robot
        self.blackboard = bt_bb.Blackboard()

    def update(self):
        self.robot.stop()
        self.blackboard.robot_stopped = True
        return py_trees.common.Status.SUCCESS

class TargetReached(py_trees.behaviour.Behaviour):
    def __init__(self,
                 robot: UniversalRobots_RTDE,
                 target_pose: list,
                 tolerance: float = 0.01,
                 name: str = "TargetReached"):
        super().__init__(name)
        self.robot = robot
        self.target_pose = target_pose
        self.tolerance = tolerance
        self.blackboard = bt_bb.Blackboard()

    def update(self):
        current = self.robot.GetActualTCPPosition()
        if all(abs(c - t) < self.tolerance for c, t in zip(current, self.target_pose)):
            self.blackboard.target_reached = True
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
