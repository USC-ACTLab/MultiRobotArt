from crazyflie_py import Crazyswarm
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import approximate_taylor_polynomial
from crazyflie_py.uav_trajectory import Trajectory
from crazyflie_py.generate_trajectory import *
from pathlib import Path
from crazyflie_py import plot_trajectory
import sys

LEADER_ID = 'cf24'
RADIUS = 0.5
DURATION = np.pi
HZ = 50

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    leader = allcfs.crazyfliesByName[LEADER_ID]
    followers = [allcfs.crazyfliesByName[k] for k in allcfs.crazyfliesByName.keys() if k != LEADER_ID]

    # Compute Phase Offset
    phase_offsets = np.linspace(0, 2*np.pi, len(followers), endpoint=False)
    # print(phase_offsets)
    
    # Takeoff:
    for cf in followers:
        cf.takeoff(1.0, 2.0)
    
    timeHelper.sleep(5.0)
    
    # Change LED color to initiate orbiting?
    t = 0
    while t <= HZ * 30:
        leader_pos = allcfs.positions[1]
        print(leader_pos)
        for i, cf in enumerate(followers):
            circle_position = (RADIUS * np.cos(t*1/HZ * 2*np.pi / DURATION + phase_offsets[i]),  RADIUS * np.sin(t*1/HZ * 2*np.pi /DURATION + phase_offsets[i]))
            desired_pos = (leader_pos[0]+ circle_position[0], leader_pos[1] + circle_position[1], max(leader_pos[2] - 0.25, 0.5))
            cf.cmdPosition(desired_pos)
            # print(desired_pos)
        t += 1
        timeHelper.sleepForRate(HZ)

    #TODO Return to start? Fix Duration?
    for cf in allcfs.crazyflies:
        cf.notifySetpointsStop()
    timeHelper.sleep(1.0)

    for cf in allcfs.crazyflies:
        cf.land(0.03, 2.0)
    
if __name__ == "__main__":
    main()