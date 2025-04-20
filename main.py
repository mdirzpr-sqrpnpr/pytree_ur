import argparse
import time
import py_trees
from py_trees.blackboard import Blackboard
import py_trees.display

from ur_robot.ur_rtde import UniversalRobots_RTDE
from bt_py.bt_builder import build_behavior_tree_from_yaml

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--bt-struct", 
        default="bt_py/sim_example.yaml",
        help="Path to BT YAML file"
    )
    parser.add_argument(
        "--ip",
        default="127.0.0.1",
        required=False,
        help="Robot IP address"
    )
    parser.add_argument(
        "--port",
        default="502",
        help="Modbus port"
    )
    parser.add_argument(
        "--simulation", 
        action="store_true", 
        help="Run tree in simulation mode"
    )
    args = parser.parse_args()

    # Initialize robot
    robot = None
    if not args.simulation:
        robot = UniversalRobots_RTDE(args.ip, args.port)

    # Build behavior tree (simulation flag passed through)
    print("[INFO] Setting up tree...")
    root = build_behavior_tree_from_yaml(
        args.bt_struct,
        robot=robot,
        simulation=args.simulation
    )

    # Setup and run
    tree = py_trees.trees.BehaviourTree(root)
    tree.setup(timeout=15)

    # Export DOT and PNG of the tree structure
    dot_graph = py_trees.display.dot_tree(root)
    with open("tree_structure.dot", "w") as dot_file:
        dot_file.write(str(dot_graph))

    try:
        import subprocess
        subprocess.run(["dot", "-Tpng", "tree_structure.dot", "-o", "tree_structure.png"], check=True)
        print("[INFO] Tree visualization saved as tree_structure.png")
    except Exception as e:
        print("[WARN] Couldn't create PNG from DOT:", e)

    print(py_trees.display.unicode_tree(root))

    try:
        while root.status != py_trees.common.Status.SUCCESS and \
              root.status != py_trees.common.Status.FAILURE:
            tree.tick()
            print("[TICK] Tree ticked.")
            print(py_trees.display.unicode_tree(root))
            print("[BLACKBOARD]", Blackboard().__dict__)
            time.sleep(0.5)  # Optional for readability
    except KeyboardInterrupt:
        if robot:
            robot.stop()
        print("\n[INFO] Stopped by user (Ctrl+C)\n")

if __name__ == "__main__":
    main()