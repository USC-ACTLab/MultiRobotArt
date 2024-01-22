from collections import namedtuple
from types import SimpleNamespace
import numpy as np
from PIL import ImageColor
import rclpy
from crazyflieLoggers import *
import rowan


Hz = 20

###
#  Landing/Takeoff commands
###

def takeoff(groupState, height=1.0, duration=2.0):
    """Takeoff command, runs for every crazyflie in groupState

    Args:
        groupState GroupState: crazyflies and timehelper objects
        height (float, optional): height to takeoff to. Defaults to 1.0.
        duration (float, optional): duration of command. Defaults to 2.0.
    """
    timeHelper = groupState.timeHelper
    crazyflies = groupState.crazyflies
    for cf in crazyflies:
        cf.takeoff(float(height), float(duration))
    timeHelper.sleep(duration)

def land(groupState, height=0.04, duration=2.0):
    """Land command, runs for every crazyflie in groupState

    Args:
        groupState GroupState: crazyflies and timehelper objects
        height (float, optional): height to land to. Defaults to 1.0.
        duration (float, optional): duration of command. Defaults to 2.0.
    """    
    timeHelper = groupState.timeHelper
    crazyflies = groupState.crazyflies
    for cf in crazyflies:
        cf.land(float(height), float(duration))
    timeHelper.sleep(duration)

###
#  Motion Primitive Commands
###

def goto_velocity(groupState, x, y, z, v, rel=False):
    """goTo command at a given *average* velocity, runs for every crazyflie in groupState

    Args:
        groupState GroupState: crazyflies and timehelper objects
        x float: Desired x position 
        y float: Desired y position
        z float: Desired z position
        v float: Desired average velocity
        rel (bool, optional): Whether or not the command is relative or absolute. Defaults to False.
    """    
    timeHelper = groupState.timeHelper
    crazyflies = groupState.crazyflies
    max_duration = 0
    for cf in crazyflies:
        curr_pos = cf.position()
        dist = np.linalg.norm(np.array(curr_pos), np.array([x, y, z]))
        duration = dist / v
        cf.goTo((float(x), float(y), float(z)), 0, duration=duration, relative=rel)
        max_duration = max(duration, max_duration)
    timeHelper.sleep(max_duration)

def goto_duration(groupState, x, y, z, duration, rel=False):
    """goTo command at a given duration, runs for every crazyflie in groupState

    Args:
        groupState GroupState: crazyflies and timehelper objects
        x float: Desired x position 
        y float: Desired y position
        z float: Desired z position
        duration float: duration of command
        rel (bool, optional): Whether or not the command is relative or absolute. Defaults to False.
    """      
    timeHelper = groupState.timeHelper
    crazyflies = groupState.crazyflies
    for cf in crazyflies:
        cf.goTo((float(x), float(y), float(z)), 0, duration=duration, relative=rel)
    timeHelper.sleep(duration)

def goto_velocity_relative_position(groupState, x, y, z, v):
    """goTo command with given average velocity and relative position

    Args:
        groupState GroupState: crazyflies and timehelper objects
        x float: Desired x position 
        y float: Desired y position
        z float: Desired z position
        v float: average velocity 
    """    
    goto_velocity(groupState, x, y, z, v, True)

def goto_duration_relative(groupState, x, y, z, duration):
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
        position = cf.position()
        cf.goTo(position, 0, 1.0)
    
###
#  Low Level Commands...
###
def getPosition(cf):
    return cf.position()

#TODO: Enable low level commands...
def cmdPosition(cf, pos):
    cf.cmdPosition(pos)

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
    fy = lambda t: radius * (np.cos(t * velocity) - 1)
    fz = lambda t: clockwise * radius * np.sin(t * velocity)

    timesteps = np.arange(0, radians/velocity, 1/Hz)
    initialPositions = [cf.position() for cf in crazyflies]
    for t in timesteps:
        for initPos, cf in zip(initialPositions, crazyflies):
            pos = np.array([fx(t), fy(t), fz(t)])
            pos += np.array(initPos)
            cf.cmdPosition(pos)
        timeHelper.sleepForRate(Hz)

    for cf in crazyflies:
        cf.notifySetpointsStop()

    # for cf in crazyflies:
    #     cf.goTo(cf.position(), 0, 1)


# Trajectory Modifiers...

def negate(groupState, command):
    originalGroupState = groupState
    simCrazyflies = []
    for cf in groupState.crazyflies:
        rclpy.spin_once(cf.node)
        simCrazyflies.append(CrazyflieSimLogger(cf.position()))
    simTimeHelper = TimeHelperSimLogger()
    simTimeHelper.currTime = originalGroupState.timeHelper.time()
    groupState = SimpleNamespace(crazyflies=simCrazyflies, timeHelper=simTimeHelper)
    # Execute Simulated Commands
    command(groupState)
    # Process commands to add startTime
    setStartTimes(groupState)
    # negate every command
    for simcf, cf in zip(groupState.crazyflies, originalGroupState.crazyflies):
        rclpy.spin_once(cf.node)
        initialPosition = cf.position()
        for i, command in enumerate(simcf.commands):
            cfCommand = command.command
            if cfCommand == 'goTo' or cfCommand == 'cmdPos' or cfCommand == 'takeoff' or cfCommand == 'land':
                # Negate the direction (Note, do not negate land...)
                if command.relative == False:
                    originalDestination = command.position
                    # (Start - goal) gives, vector from start, so add back starting position for 2*start - goal
                    negatedDestination = 2 * np.array(initialPosition) - np.array(originalDestination)
                    simcf.commands[i].position = negatedDestination
                    # print(negatedDestination, originalDestination, initialPosition)
                else:
                    originalDestination = command.position
                    negatedDestination = -originalDestination
                    simcf.commands[i].position = negatedDestination
            else:
                # Just run the original command, no need to alter anything
                pass
    execute_commands(groupState, originalGroupState)

def rotate(groupState, command, x_rot, y_rot, z_rot):
    originalGroupState = groupState
    simCrazyflies = []
    for cf in groupState.crazyflies:
        rclpy.spin_once(cf.node)
        simCrazyflies.append(CrazyflieSimLogger(cf.position()))
    simTimeHelper = TimeHelperSimLogger()
    simTimeHelper.currTime = originalGroupState.timeHelper.time()
    groupState = SimpleNamespace(crazyflies=simCrazyflies, timeHelper=simTimeHelper)
    # Execute Simulated Commands
    command(groupState)
    # Process commands to add startTime
    setStartTimes(groupState)
    for simcf, cf in zip(groupState.crazyflies, originalGroupState.crazyflies):
        rclpy.spin_once(cf.node)
        initialPosition = cf.position()
        for i, command in enumerate(simcf.commands):
            cfCommand = command.command
            if cfCommand == 'goTo' or cfCommand == 'cmdPos':
                if command.relative == False:
                    originalDestination = command.position
                    directionVector = np.array(initialPosition) - np.array(originalDestination)
                    rotQuat = rowan.from_euler(x_rot, y_rot, z_rot, convention='xyz')
                    rotatedVector = rowan.rotate(rotQuat, directionVector)
                    newDestination = rotatedVector + initialPosition
                    simcf.commands[i].position = newDestination
                else:
                    originalDestination = command.position
                    directionVector = np.array(initialPosition) - np.array(originalDestination)
                    rotQuat = rowan.from_euler(x_rot, y_rot, z_rot, convention='xyz')
                    rotatedVector = rowan.rotate(rotQuat, directionVector)
                    newDestination = rotatedVector
                    simcf.commands[i].position = newDestination
            else:
                # Just run the original command, no need to alter anything
                pass
    execute_commands(groupState, originalGroupState)

def stretch(groupState, command, x_stretch, y_stretch, z_stretch, time_stretch):
    originalGroupState = groupState
    simCrazyflies = []
    for cf in groupState.crazyflies:
        rclpy.spin_once(cf.node)
        simCrazyflies.append(CrazyflieSimLogger(cf.position()))
    simTimeHelper = TimeHelperSimLogger()
    simTimeHelper.currTime = originalGroupState.timeHelper.time()
    groupState = SimpleNamespace(crazyflies=simCrazyflies, timeHelper=simTimeHelper)
    # Execute Simulated Commands
    command(groupState)
    # Process commands to add startTime
    setStartTimes(groupState)
    for simcf, cf in zip(groupState.crazyflies, originalGroupState.crazyflies):
        rclpy.spin_once(cf.node)
        initialPosition = cf.position()
        for i, command in enumerate(simcf.commands):
            cfCommand = command.command
            if cfCommand == 'goTo' or cfCommand == 'cmdPos' or cfCommand == 'takeoff' or cfCommand == 'land':
                if command.relative == False:
                    originalDestination = command.position
                    directionVector = np.array(initialPosition) - np.array(originalDestination)
                    stretchedVector = directionVector.multiply(np.array((x_stretch, y_stretch, z_stretch)))
                    newDestination = stretchedVector + initialPosition
                    simcf.commands[i].position = newDestination
                    simcf.commands[i].duration *= time_stretch
                else:
                    originalDestination = command.position
                    directionVector = np.array(initialPosition) - np.array(originalDestination)
                    newDestination = directionVector.multiply(np.array((x_stretch, y_stretch, z_stretch)))
                    simcf.commands[i].position = newDestination
                    simcf.commands[i].duration *= time_stretch
            else:
                # Just run the original command, no need to alter anything
                pass
    execute_commands(groupState, originalGroupState)

def addTrajectories(groupState, command1, command2):
    # TODO: Need to convert goTo's to low level commands with formula
    pass

def subtractTrajectories(groupState, command1, command2):
    pass

def execute_commands(simGroupState, originalGroupState):
    # Note, assumes all crazyflies in group are issued similar commands (all are told goTo)
    # Assumes first command is to be executed immediately
    # If not the case, different groups should be used

    commandsForSingleCF = simGroupState.crazyflies[0].commands
    highLevel = True

    for i, command in enumerate(commandsForSingleCF):
        if highLevel and command.command == 'cmdPos':
            highLevel = False
        if not highLevel and command.command != 'cmdPos':
            highLevel = True
            for realCF in originalGroupState.crazyflies:
                realCF.notifySetpointsStop()
        for simCF, realCF in zip(simGroupState.crazyflies, originalGroupState.crazyflies):
            execute_single_command(simCF.commands[i], realCF)
        originalGroupState.timeHelper.sleep(command.duration)

def execute_single_command(command, crazyflie):
    if command.command == 'goTo':
        crazyflie.goTo(command.position, command.yaw, command.duration, command.relative)
    elif command.command == 'cmdPos':
        crazyflie.cmdPosition(command.position)
    elif command.command == 'takeoff':
        crazyflie.takeoff(command.height, command.duration)
    elif command.command == 'land':
        crazyflie.land(command.height, command.duration)
    elif command.command == 'setLEDColor':
        crazyflie.setLEDColor(*command.color)
    elif command.command == 'position':
        pass
    elif command.command == 'notifySetpointsStop':
        crazyflie.notifySetpointsStop()


