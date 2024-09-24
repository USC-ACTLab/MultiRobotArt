import yaml
from ament_index_python.packages import get_package_share_directory
from crazyflie_py import Crazyswarm
import rclpy
import threading
from rclpy.node import Node
class crazyflie_node(Node):
    def __init__(self, crazyswarm):
        super().__init__("crazyswarm_node")
        Hz = 20
        self.crazyswarm = crazyswarm
        self.timeHelper = crazyswarm.timeHelper
        self.timer = self.create_timer(
            1 / Hz,
            self.timer_callback,
        )

    def timer_callback(self):
        self.timeHelper.sleepForRate(50)
# Inject Imports Here:

def launch(nodes):
    executor = rclpy.executors.MultiThreadedExecutor()

    for node in nodes:
        executor.add_node(node)
    
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        while rclpy.ok():
            pass
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
    thread.join()
    

def main():
    # node_config_file = get_package_share_directory('crazyflie'), 'config', 'crazyflies.yaml'
    
    # Initialize swarm
    swarm = Crazyswarm()
    with open("cfs_ordering.yaml") as f:
        ordering = yaml.safe_load(f) #TODO!!!
        order = ordering['cfs']
    
    crazyflies = [swarm.allcfs.crazyfliesById[int(k)] for k in order]
    counter = 0
    nodes = []
    cfs = []

    #   -----------Insert Nodes Here----------- 

    nodes.append(crazyflie_node(swarm))
    # Launch all nodes
    return launch(nodes)

if __name__ == '__main__':
    main()
