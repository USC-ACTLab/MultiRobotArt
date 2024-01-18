import numpy as np
from PIL import ImageColor

Hz = 20

###
#  Landing/Takeoff commands
###

def takeoff(groupState, height=1.0, duration=2.0):
    '''
    Takeoff to a certain height for every crazyflie in groupState
    '''
    timeHelper = groupState.timeHelper
    crazyflies = groupState.crazyflies
    for cf in crazyflies:
        cf.takeoff(float(height), float(duration))
    timeHelper.sleep(duration)

def land(groupState, height=0.04, duration=2.0):
    timeHelper = groupState.timeHelper
    crazyflies = groupState.crazyflies
    for cf in crazyflies:
        cf.land(float(height), float(duration))
    timeHelper.sleep(duration)

###
#  Motion Primitive Commands
###

def goto_at_speed(groupState, x, y, z, v, rel=False):
    timeHelper = groupState.timeHelper
    crazyflies = groupState.crazyflies
    max_duration = 0
    for cf in crazyflies:
        curr_pos = cf.getPosition()
        dist = np.linalg.norm(curr_pos, np.array([x, y, z]))
        duration = dist / v
        cf.goTo((float(x), float(y), float(z)), 0, duration=duration, relative=rel)
        max_duration = max(duration, max_duration)
    timeHelper.sleep(max_duration)

def goto_duration(groupState, x, y, z, duration, rel=False):
    timeHelper = groupState.timeHelper
    crazyflies = groupState.crazyflies
    for cf in crazyflies:
        cf.goTo((float(x), float(y), float(z)), 0, duration=duration, relative=rel)
    timeHelper.sleep(duration)

def goto_rel_at_speed(groupState, x, y, z, v):
    goto_at_speed(groupState, x, y, z, v, True)

def goto_rel_duration(groupState, x, y, z, duration):
    goto_duration(groupState, x, y, z, duration, True)

def move_direction(groupState, direction, distance, duration):
    timeHelper = groupState.timeHelper
    crazyflies = groupState.crazyflies
    for cf in crazyflies:
        pos = cf.getPosition()
        if direction == "up":
            cf.goTo((0, 0, distance), 0, duration, relative=True)
        elif direction == "forward":
            cf.goTo((distance, 0, 0), 0, duration=duration, relative=True)
        elif direction == "backward":
            cf.goTo((-distance, 0, 0), 0, duration=duration, relative=True)
        elif direction == "down":
            cf.goTo((0, 0, -distance), 0, duration, relative=True)
    timeHelper.sleep(duration)
    return duration

def stop_and_hover(groupState, height=None):

    crazyflies = groupState.crazyflies
    for cf in crazyflies:
        position = cf.getPosition()
        cf.goTo(position, 0, 1.0)
    


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

def setLEDColorFromHex(groupState, hex):
    crazyflies = groupState.crazyflies
    rgb = ImageColor.getcolor(hex, "RGB")
    for cf in crazyflies:
        cf.setLEDColor(*rgb)

def setLEDColor(groupState, r, g, b):
    crazyflies = groupState.crazyflies
    for cf in crazyflies:
        cf.setLEDColor(r, g, b)

### 
#  Trajectory commands...
###


def circle(groupState, radius, velocity, radians, direction):
    crazyflies = groupState.crazyflies
    timeHelper = groupState.timeHelper
    clockwise = 1
    if direction == 'ccw':
        clockwise = -1
    
    fx = lambda t: 0
    fy = lambda t: radius * np.cos(t * velocity)
    fz = lambda t: clockwise * radius * np.sin(t * velocity)

    timesteps = np.linspace(0, radians/velocity, Hz)
    initialPositions = [cf.getPosition() for cf in crazyflies]
    for t in timesteps:
        for initPos, cf in zip(initialPositions, crazyflies):
            pos = np.array(fx(t), fy(t), fz(t))
            pos += np.array(initPos)
            cf.cmdPos(pos)
        timeHelper.sleepForRate(Hz)

    for cf in crazyflies:
        cf.notifySetpointsStop()


###
#   Logger Class
###



class CrazyflieSimLogger:
    
    def __init__(self):
        self.hl_commands = []
        self.ll_commands = []

    def goTo(self, pos, yaw, duration, relative=False):
        self.hl_commands.append(('goTo', pos, yaw, duration, relative))
    
    def cmdPos(self, pos):
        self.ll_commands.append(('cmdPos', pos))
    
    def setLEDColor(self):
        pass
    
    def takeoff(self):
        pass

    def land(self):
        pass

    def getPosition(self):
        pass

    def notifySetpointsStop(self):
        pass

class TimeHelperSimLogger:
    def __init__(self):
        self.times = []
        self.currTime = 0

    def sleep(self, duration):
        self.times.append(self.currTime)
        self.currTime += duration
    
    def sleepForRate(self, Hz):
        self.times.append(self.currTime)
        self.currTime += 1/Hz
    
    def sleepUntil(self, time):
        self.times