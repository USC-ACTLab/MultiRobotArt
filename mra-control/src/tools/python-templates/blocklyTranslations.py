import numpy as np
from PIL import ImageColor

###
#  Landing/Takeoff commands
###

def takeoff(cf, height=1.0, duration=2.0):
    cf.takeoff(float(height), float(duration))
    return duration

def land(cf, height=0.04, duration=2.0):
    cf.land(float(height), float(duration))
    return duration

###
#  Motion Primitive Commands
###

def goto_at_speed(cf, x, y, z, v, rel=False):
    curr_pos = cf.getPosition()
    dist = np.linalg.norm(curr_pos, np.array([x, y, z]))
    duration = dist / v
    cf.goTo((float(x), float(y), float(z)), 0, duration=duration, relative=rel)
    return duration

def goto_duration(cf, x, y, z, duration, rel=False):
    cf.goTo((float(x), float(y), float(z)), 0, duration=duration, relative=rel)
    return duration

def goto_rel_at_speed(cf, x, y, z, v):
    return goto_at_speed(cf, x, y, z, v, True)

def goto_rel_duration(cf, x, y, z, duration):
    return goto_duration(cf, x, y, z, duration, True)

def move_direction(cf, direction, distance, duration):
    if direction == "up":
        cf.goTo(0, 0, distance, duration, relative=True)
    elif direction == "forward":
        cf.goTo(distance, 0, 0, duration=duration, relative=True)
    elif direction == "backward":
        cf.goTo(-distance, 0, 0, duration=duration, relative=True)
    elif direction == "down":
        cf.goTo(0, 0, -distance, duration, relative=True)
    return duration

#TODO Check on cf.getPosition()
def stop_and_hover(cf, height=None):
    position = cf.getPosition()
    goal_pos = position
    if height != None:
        goal_pos[2] = height
        duration = 2.0
    else:
        duration = 0.1
    cf.goTo(goal_pos, duration=duration)
    return duration
    


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
# Colors
###

def setLEDFromHex(cf, hex):
    rgb = ImageColor.getcolor(hex, "RGB")
    cf.setLEDColor(*rgb)

### 
#  Trajectory commands...
###

def circle():
    pass