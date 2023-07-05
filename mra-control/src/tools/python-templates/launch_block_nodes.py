import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from crazyflie_py import Crazyswarm
import rclpy
import threading

# Inject Imports Here:

def launch(nodes):
    threads = []
    for node in nodes:
        thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
        thread.start()
        threads.append(thread)
    while not all([n.done for n in nodes]):
        pass
    rclpy.shutdown()
    [t.join() for t in threads]

def main():
    # node_config_file = get_package_share_directory('crazyflie'), 'config', 'crazyflies.yaml'
    
    # Initialize swarm
    swarm = Crazyswarm()
    with open("cfs_ordering.yaml") as f:
        ordering = yaml.load(f)
        order = ordering['cfs']
    # all_crazyflies = swarm.allcfs.crazyflies
    all_crazyflies = [swarm.allcfs.crazyfliesById[k] for k in order]
    # Construct Launch Description from yaml files
    # One node per line in the timeline, linked to a worker node described in the yaml file
    counter = 0
    nodes = []
    cfs = []

    # Inject append here...

    # cfs.append(all_crazyflies[0])
    # nodes.append(group1_node.worker_node(cfs, len(nodes), 1))
    
    # Launch all nodes
    return launch(nodes)

if __name__ == '__main__':
    main()
