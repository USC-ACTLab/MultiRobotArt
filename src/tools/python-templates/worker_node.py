import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from crazyflie_py import generate_trajectory
import numpy as np
from blocklyTranslations import *

Hz = 30

class worker_node(Node):

    def __init__(self, crazyflies, id=0, num_nodes=1):
        """
        id: a unique id between 0 and num_nodes corresponding to the thread number of this worker
        num_nodes: number of nodes (threads) in total
        """
        super().__init__("worker_node_{}".format(id))
        self.id = id
        self.num_nodes = num_nodes
        self.crazyflies = crazyflies

        self.execution_ready_subscription(
            Int32,
            'ready',
            self.ready_callback,
            num_nodes + 1
        )
        self.execution_ready_publisher(
            Int32,
            'ready',
            num_nodes + 1
        )
        self.timer = self.create_timer(1/Hz, self.timer_callback)
        self.ready_ids = set()
        self.executing = False
        self.running = False
        self.done = False
    
    def compute_trajectories(self):
        """
        Inject Trajectory computation code here...
        """
        trajectories = []
        
        #TODO insert trajectories here...

        return trajectories

    def upload_trajectories(crazyflies, trajectories):
        '''
            Upload trajectories to crazyflies one by one
        '''

        # TODO: Currently doesn't support overlapping crazyflies as we will overwrite trajectories...
        for i, traj in enumerate(trajectories):
            for cf in crazyflies:
                cf.uploadTrajectory(traj, i, 0)

    def begin(self):
        """
            Prepare for execution. Pre-compute trajectories and upload them to crazyflies
        """
        trajectories = self.compute_trajectories()
        self.upload_trajectories(trajectories)
        msg = Int32()
        msg.data = self.id
        self.ready_publisher.publish(msg)

    def time(self):
        return self.get_clock().now().nanoseconds / 1e9
    
    def execute_blocks(self):
        """
        Must be injected into...

        Typical format should be:
        
        start_time = 0.0
        duration = 3.0
        self.wait_until(start_time)
        for cf in self.crazyflies:
            cf.takeoff(1.0, duration)
        self.wait_until(start_time+duration)

        where start_time, duration, and in the inner portion of the for loop are provided.
        """
        # BLOCKS...
        pass

    def ready_callback(self, msg):
        self.ready_ids.add(msg.data)
    
    def timer_callback(self):
        if not self.running:
            self.begin()
            self.running = True
        if len(self.ready_ids) == self.num_nodes and not self.executing:
            self.running = True
            self.start_time = self.get_clock().now()
            self.execute_blocks()
            self.destroy_node()
            self.done = True

    def wait_until(self, end_time):
        while self.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0)

    def wait(self, time):
        end_time = self.time() + time
        self.wait_until(end_time)