# import yaml
# import py_trees

# from bt_py import bt_nodes, bt_nodes_sim

# def build_behavior_tree_from_yaml(yaml_path: str, robot=None, simulation=False) -> py_trees.behaviour.Behaviour:
#     with open(yaml_path, 'r') as file:
#         bt_data = yaml.safe_load(file)

#     nodes_def = bt_data['nodes']

#     # Use simulation nodes or real nodes
#     node_module = bt_nodes_sim if simulation else bt_nodes

#     def create_node(name):
#         node_info = nodes_def[name]
#         node_type = node_info['type']
#         params = node_info.get('params', {})

#         # Compose the class name dynamically based on node_type
#         cls_name = node_type + ("Sim" if simulation else "")
#         NodeClass = getattr(node_module, cls_name)

#         # Check if robot is required (all current nodes use it)
#         if 'pose' in params or 'joints' in params or node_type not in ['Sequence', 'Selector', 'Parallel']:
#             return NodeClass(robot=robot, name=name, **params)
#         else:
#             return NodeClass(name=name, **params)

#     # Recursive tree builder
#     def build_subtree(node_name):
#         node_info = nodes_def[node_name]
#         node_type = node_info['type']

#         if node_type in ['Sequence', 'Selector', 'Parallel']:
#             children = [build_subtree(child_name) for child_name in node_info['children']]
#             if node_type == 'Sequence':
#                 composite = py_trees.composites.Sequence(name=node_name, memory=False)
#             elif node_type == 'Selector':
#                 composite = py_trees.composites.Selector(name=node_name, memory=False)
#             elif node_type == 'Parallel':
#                 # Use success_on_all and fail_on_one for parallel execution
#                 success_policy = py_trees.common.ParallelPolicy.SuccessOnAll()
#                 failure_policy = py_trees.common.ParallelPolicy.FailureOnOne()
#                 composite = py_trees.composites.Parallel(
#                     name=node_name,
#                     policy=py_trees.common.ParallelPolicy(
#                         success=success_policy,
#                         failure=failure_policy
#                     )
#                 )
#             composite.add_children(children)
#             return composite
#         else:
#             return create_node(node_name)

#     root_node = build_subtree(bt_data['root'])
#     return root_node

import yaml
import py_trees

from bt_py import bt_nodes, bt_nodes_sim


def build_behavior_tree_from_yaml(yaml_path: str, robot=None, simulation=False) -> py_trees.behaviour.Behaviour:
    with open(yaml_path, 'r') as file:
        bt_data = yaml.safe_load(file)

    nodes_def = bt_data['nodes']

    # Use simulation nodes or real nodes
    node_module = bt_nodes_sim if simulation else bt_nodes

    def create_node(name):
        node_info = nodes_def[name]
        node_type = node_info['type']
        params = node_info.get('params', {})

        # Compose the class name dynamically based on node_type
        cls_name = node_type + ("Sim" if simulation else "")
        NodeClass = getattr(node_module, cls_name)

        # Check if robot is required (all current nodes use it)
        if 'pose' in params or 'joints' in params or node_type not in ['Sequence', 'Selector', 'Parallel', 'Retry']:
            return NodeClass(robot=robot, name=name, **params)
        else:
            return NodeClass(name=name, **params)

    # Recursive tree builder
    def build_subtree(node_name):
        node_info = nodes_def[node_name]
        node_type = node_info['type']

        if node_type in ['Sequence', 'Selector', 'Parallel']:
            children = [build_subtree(child_name) for child_name in node_info['children']]
            if node_type == 'Sequence':
                composite = py_trees.composites.Sequence(name=node_name, memory=False)
            elif node_type == 'Selector':
                composite = py_trees.composites.Selector(name=node_name, memory=False)
            elif node_type == 'Parallel':
                # Use success_on_all and fail_on_one for parallel execution
                success_policy = py_trees.common.ParallelPolicy.SuccessOnAll()
                failure_policy = py_trees.common.ParallelPolicy.FailureOnOne()
                composite = py_trees.composites.Parallel(
                    name=node_name,
                    policy=py_trees.common.ParallelPolicy(
                        success=success_policy,
                        failure=failure_policy
                    )
                )
            composite.add_children(children)
            return composite

        elif node_type == 'Retry':
            child_name = node_info['children'][0]
            child = build_subtree(child_name)
            num_failures = node_info.get('params', {}).get('num_failures', 1)
            decorator = py_trees.decorators.Retry(name=node_name, child=child, num_failures=num_failures)
            return decorator

        else:
            return create_node(node_name)

    root_node = build_subtree(bt_data['root'])
    return root_node