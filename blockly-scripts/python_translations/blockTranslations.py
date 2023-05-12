import numpy as np

# Motion Primitive Commands

def goto_at_speed(cf, x, y, z, v):
    curr_pos = cf.getPosition()
    dist = np.linalg.norm(curr_pos, np.array([x, y, z]))
    duration = dist / v
    cf.goTo(x, y, z, duration=duration)

def goto_duration(cf, x, y, z, duration):
    cf.goTo(x, y, z, duration=duration)


# Trajectory commands...