import numpy as np

###
#  Landing/Takeoff commands
###

def takeoff(cf, height=1.0, duration=2.0):
    cf.takeoff(height, duration)

def land(cf, height=0.04, duration=2.0):
    cf.land(height, duration)

###
#  Motion Primitive Commands
###

def goto_at_speed(cf, x, y, z, v, rel=False):
    curr_pos = cf.getPosition()
    dist = np.linalg.norm(curr_pos, np.array([x, y, z]))
    duration = dist / v
    cf.goTo(x, y, z, duration=duration, rel=rel)

def goto_duration(cf, x, y, z, duration, rel=False):
    cf.goTo(x, y, z, duration=duration, rel=rel)

def goto_rel_at_speed(cf, x, y, z, v):
    goto_at_speed(cf, x, y, z, v, True)

def goto_rel_duration(cf, x, y, z, duration):
    goto_duration(cf, x, y, z, duration, True)

#TODO Check on cf.getPosition()
def stop_and_hover(cf, height=None):
    position = cf.getPosition()
    goal_pos = position
    if height != None:
        goal_pos[2] = height
    cf.goTo(goal_pos, duration=2.0)

###
#  Low Level Commands...
###
def getPosition(cf):
    return cf.getPosition()

#TODO: Enable low level commands...
def cmdPos(cf, pos):
    cf.cmdPos(pos)

def enableHighLevelCommander(cf):
    cf.notifySetpointsStop()

### 
#  Trajectory commands...
###

def circle():
    pass