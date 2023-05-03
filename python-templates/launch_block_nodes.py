import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from crazyflie_py import CrazySwarm


def generate_launch_description():
    # TODO: Change package name to correct description
    node_config_file = get_package_share_directory('my_package'), 'config', 'node_config.yaml'
    with open(node_config_file, 'r') as f:
        node_configs = yaml.load(f)

    # Construct Launch Description from yaml files
    # One node per line in the timeline, linked to a worker node described in the yaml file
    nodes = node_configs['nodes']
    description = []
    for node_info in nodes:
        description.append(Node(node_info))
    
    # Initialize swarm
    swarm = CrazySwarm()
    all_crazyflies = swarm.allcfs.crazyflies

    # Launch all nodes
    return LaunchDescription(description)