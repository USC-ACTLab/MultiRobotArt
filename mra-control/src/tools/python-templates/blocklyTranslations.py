from collections import namedtuple
from types import SimpleNamespace
import numpy as np
from PIL import ImageColor
import rclpy
from crazyflieLoggers import *
import rowan
from scipy.spatial.transform import Rotation as R

import traceback

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
        # try:
        #     if not isinstance(cf, CrazyflieSimLogger):
        #         rclpy.spin_once(cf.node)
        # except Exception:
        #     cf.node.get_logger().error(traceback.format_exc())
        #     pass
        curr_pos = cf.position()
        dist = np.linalg.norm(np.array(curr_pos) - np.array([x, y, z]))
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
    # return
    crazyflies = groupState.crazyflies
    rgb = ImageColor.getcolor(hex, "RGB")
    for cf in crazyflies:
        cf.setLEDColor(*rgb)

def setLEDColor(groupState, r, g, b):
    # return
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

# Trajectory Modifiers...

def negate(groupState, command):
    originalGroupState = groupState
    simCrazyflies = []
    for cf in groupState.crazyflies:
        # try:
        #     if not isinstance(cf, CrazyflieSimLogger):
        #         rclpy.spin_once(cf.node)
        # except Exception:
        #     cf.node.get_logger().error(traceback.format_exc())
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
        # try:
        #     rclpy.spin_once(cf.node)
        # except Exception:
        #     cf.node.get_logger().error(traceback.format_exc())
        #     print('negate2')
        #     pass
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
    simGroupState = simCommand(groupState, command)
    while x_rot > np.pi:
        x_rot -= 2*np.pi
    while y_rot > np.pi:
        y_rot -= 2*np.pi
    while z_rot > 2*np.pi:
        z_rot -= 2*np.pi
    r = R.from_euler('xyz', (x_rot, y_rot, z_rot), degrees=False)

    for simcf, cf in zip(simGroupState.crazyflies, groupState.crazyflies):
        # try:
        #     if not isinstance(cf, CrazyflieSimLogger):
        #         rclpy.spin_once(cf.node)
        # except Exception:
        #     cf.node.get_logger().error(traceback.format_exc())
        initialPosition = cf.position()
        for i, command in enumerate(simcf.commands):
            cfCommand = command.command
            if cfCommand == 'goTo' or cfCommand == 'cmdPos':
                if command.relative == False:
                    originalDestination = command.position
                    directionVector = np.array(originalDestination) - np.array(initialPosition)
                    rotatedVector = r.apply(directionVector)
                    # rotQuat = rowan.from_euler(x_rot, y_rot, z_rot, convention='xyz')
                    # rotatedVector = rowan.rotate(rotQuat, directionVector)
                    newDestination = rotatedVector + initialPosition
                    simcf.commands[i].position = newDestination
                else:
                    originalDestination = command.position
                    directionVector = np.array(originalDestination)
                    # rotQuat = rowan.from_euler(x_rot, y_rot, z_rot, convention='xyz')
                    # rotatedVector = rowan.rotate(rotQuat, directionVector)
                    rotatedVector = r.apply(directionVector)
                    newDestination = rotatedVector
                    simcf.commands[i].position = newDestination
            else:
                # Just run the original command, no need to alter anything
                pass
    execute_commands(simGroupState, groupState)

def stretchTrajectory(groupState, command, x_stretch, y_stretch, z_stretch, time_stretch):
    simGroupState = simCommand(groupState, command)
    for simcf, cf in zip(simGroupState.crazyflies, groupState.crazyflies):
        # try:
        #     if not isinstance(cf, CrazyflieSimLogger):
        #         rclpy.spin_once(cf.node)
        # except Exception:
        #     cf.node.get_logger().error(traceback.format_exc())
        initialPosition = cf.position()
        for i, command in enumerate(simcf.commands):
            cfCommand = command.command
            if cfCommand == 'goTo' or cfCommand == 'cmdPos' or cfCommand == 'takeoff' or cfCommand == 'land':
                if command.relative == False:
                    originalDestination = command.position
                    directionVector =  np.array(originalDestination) - np.array(initialPosition)
                    stretchedVector = np.multiply(directionVector, (x_stretch, y_stretch, z_stretch)) # directionVector.multiply(np.array((x_stretch, y_stretch, z_stretch)))
                    newDestination = stretchedVector + initialPosition
                    simcf.commands[i].position = newDestination
                    simcf.commands[i].duration *= time_stretch
                else:
                    originalDestination = command.position
                    directionVector = np.array(initialPosition) - np.array(originalDestination)
                    newDestination = np.multiply(directionVector, (x_stretch, y_stretch, z_stretch))
                    simcf.commands[i].position = newDestination
                    simcf.commands[i].duration *= time_stretch
            else:
                # Just run the original command, no need to alter anything
                pass
    execute_commands(simGroupState, groupState)

def simCommand(originalGroupState, command):
    simCrazyflies = []
    for cf in originalGroupState.crazyflies:
        # try:
        #     if not isinstance(cf, CrazyflieSimLogger):
        #         rclpy.spin_once(cf.node)
        # except Exception:
        #     cf.node.get_logger().error(traceback.format_exc())
        simCrazyflies.append(CrazyflieSimLogger(cf.position()))
    simTimeHelper1 = TimeHelperSimLogger()
    simTimeHelper1.currTime = originalGroupState.timeHelper.time()
    simGroupState = SimpleNamespace(crazyflies=simCrazyflies, timeHelper=simTimeHelper1) 
    # Execute Simulated Commands
    command(simGroupState)

    # Reset position
    for cf in simGroupState.crazyflies:
        cf.currPosition = cf.initialPosition
    
    setStartTimes(simGroupState)
    return simGroupState

def addTrajectories(groupState, command1, command2):
    groupState1 = simCommand(groupState, command1)
    groupState2 = simCommand(groupState, command2)

    # Convert any goTo commands to LL Commands
    convertToLL(groupState1)
    convertToLL(groupState2)
    # Get duration of latest non-notify setpoints stop command
    if groupState1.crazyflies[0].commands[-1].command == 'notifySetpointsStop':
        finalCommand1 = -2
    else:
        finalCommand1 = -1
    if groupState2.crazyflies[0].commands[-1].command == 'notifySetpointsStop':
        finalCommand2 = -2
    else:
        finalCommand2 = -1

    # Get duration of each command
    duration1 = groupState1.crazyflies[0].commands[finalCommand1].startTime + groupState1.crazyflies[0].commands[finalCommand1].duration - groupState1.crazyflies[0].commands[0].startTime
    duration2 = groupState2.crazyflies[0].commands[finalCommand2].startTime + groupState2.crazyflies[0].commands[finalCommand2].duration - groupState2.crazyflies[0].commands[0].startTime
    if duration1 > duration2:
        # Command 1 is longer, scale 2 appropriately
        command_duration = duration1
        scaling_factor = duration1 / duration2
        for cf in groupState2.crazyflies:
            for command in cf.commands:
                    command.startTime *= scaling_factor
                    command.duration *= scaling_factor
    else:
        # Command 2 is longer, scale 1 appropriately
        command_duration = duration2
        scaling_factor = duration2 / duration1
        for cf in groupState1.crazyflies:
            for command in cf.commands:
                command.startTime *= scaling_factor
                command.duration *= scaling_factor

    # TODO: Convert to 20Hz
    
    # New commands for group1
    # for cf in groupState1.crazyflies:
    #     new_commands = []
    #     counter = 0
    #     for t in np.arange(0, max(duration1, duration2), 0.05):
    #         curr_command = cf.commands[counter]
    #         if t == curr_command.startTime:

            

    # Assume same duration for now...
    # take list of commands, gather positions, and interpolate/extrapolate
    # Should be same duration and length, so just add...
    for cf1, cf2, realCf in zip(groupState1.crazyflies, groupState2.crazyflies, groupState.crazyflies):
        # try:
        #     if not isinstance(realCf, CrazyflieSimLogger):
        #         rclpy.spin_once(realCf.node)
        # except Exception:
        #     cf.node.get_logger().error(traceback.format_exc())
        initial_position = realCf.position()
        for c1, c2 in zip(cf1.commands, cf2.commands):
            if c1.command == 'cmdPos':
                new_position = np.array(c1.position) + np.array(c2.position) - initial_position
                c1.position = new_position
        if len(cf1.commands) > len(cf2.commands):
            cf1.commands = cf1.commands[:len(cf2.commands)]
    execute_commands(groupState1, groupState)

            


def goToPolynomial(T, p, v, a, p2, v2, a2):
        poly = [0] * 8

        # Taken from crazyflie Firmware Library
        T2 = T * T
        T3 = T2 * T
        T4 = T3 * T
        T5 = T4 * T
        T6 = T5 * T
        T7 = T6 * T
        poly[0] = p
        poly[1] = v
        poly[2] = a / 2
        poly[3] = 0
        poly[4] = -(5 * (14 * p - 14 * p2 + 8 * T * v + 6 * T * v2 + 2 * T2 * a - T2 * a2)) / (2 * T4)
        poly[5] = (84 * p - 84 * p2 + 45 * T * v + 39 * T * v2 + 10 * T2 * a - 7 * T2 * a2) / T5
        poly[6] = -(140 * p - 140 * p2 + 72 * T * v + 68 * T * v2 + 15 * T2 * a - 13 * T2 * a2) / (2 * T6)
        poly[7] = (2 * (10 * p - 10 * p2 + 5 * T * v + 5 * T * v2 + T2 * a - T2 * a2)) / T7
        return lambda t: sum([poly[i]*t**i for i in range(len(poly))])

def convertToLL(groupState):
    # Convert high level command to series of low level commands
    # Typically, this is converting a goTo command to cmdPos using a 7-Degree polynomial
    for cf in groupState.crazyflies:
        # TODO: Change to correct initialPosition!
        simCF = CrazyflieSimLogger(cf.initialPosition)
        currPosition = simCF.position()
        new_commands = []
        for command in cf.commands:
            if command.command == 'goTo' or command.command == 'land' or command.command == 'takeoff':
                startTime = command.startTime
                # Convert to Polynomial
                if command.relative == True:
                    goal = np.array(command.position) + np.array(currPosition)
                else:
                    goal = command.position
                fx = goToPolynomial(command.duration, currPosition[0], 0, 0, goal[0], 0, 0)
                fy = goToPolynomial(command.duration, currPosition[1], 0, 0, goal[1], 0, 0)
                fz = goToPolynomial(command.duration, currPosition[2], 0, 0, goal[2], 0, 0)

                timesteps = np.arange(0, command.duration, 1/Hz)
                commands = [Command('cmdPos', t+startTime, (fx(t), fy(t), fz(t)), 0, 1/Hz, relative=False) for t in timesteps]
                new_commands.extend(commands)
            else:
                new_commands.append(command)
        cf.commands = new_commands

            

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
            # for realCF in originalGroupState.crazyflies:
            #     realCF.notifySetpointsStop()
                # realCF.goTo(realCF.position(), 0, 0.5)
        for simCF, realCF in zip(simGroupState.crazyflies, originalGroupState.crazyflies):
            execute_single_command(simCF.commands[i], realCF)
        originalGroupState.timeHelper.sleep(command.duration)
    if not highLevel:
        for realCF in originalGroupState.crazyflies:
            realCF.notifySetpointsStop()
        # originalGroupState.timeHelper.sleep(0.1)
        for realCF in originalGroupState.crazyflies:
            realCF.goTo(realCF.position(), 0, 2.00)
            # realCF.setLEDColor(0, 0, 0.5)
            print('appended goTo')

def execute_single_command(command, crazyflie):
    if command.command == 'goTo':
        crazyflie.goTo(command.position, command.yaw, command.duration, command.relative)
    elif command.command == 'cmdPos':
        # print(command.position)
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
        pass

