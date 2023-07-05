#


from crazyflie_py import Crazyswarm
import matplotlib.pyplot as plt
import numpy as np
from crazyflie_py.uav_trajectory import Trajectory
from crazyflie_py.generate_trajectory import *
from crazyflie_py import Crazyswarm

class Block:
    
    @property
    def requires_trajectory(self):
        requiresTrajectory = False
    
    @property
    def functions(self):
        return None

def main(crazyflies=None, blocks=[]):
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    
    if crazyflies is None:
        crazyflies = allcfs.crazyflies
    
    """
    Generate and upload trajectories
    Perform any precomputation needed to make 4D piece-wise polynomial trajectories
    """
    trajectories = []
    
    for block in blocks:
        if block.requiresTrajectory:
            # TODO: Check if file already exists in data/ dir
            fx, fy, fz, fyaw, domain = block.functions
            pos_data = generate_position_data(fx, fy, fz, fyaw, domain)
            #TODO make num_pieces adaptive?
            traj = generate_trajectory(pos_data, num_pieces)
            trajectories.append(traj)
            block.trajectoryID = i
    
    for i, traj in enumerate(trajectories):
        for cf in crazyflies:
            cf.uploadTrajectory(i, 0, traj)
    """
    SYNCHRONIZE CLOCKS
    TODO: make subscriber or callbacks for when each node is ready to start executing
    """

    """
    RUN BLOCKS
    """
    


if __name__ == "__main__":
    main()

