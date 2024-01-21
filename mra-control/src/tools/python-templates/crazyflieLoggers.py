
from typing import Any, Collection
from dataclasses import dataclass
from rclpy.node import Node
import string 
import random

Hz = 20
###
#   Logger Classes
###

@dataclass
class Command:
    command: Any = None
    startTime: float = 0.0
    position: Collection = (0, 0, 0)
    yaw: float = 0
    duration: float = 0
    relative: bool = False
    height: float = 0
    color: Collection = (0, 0, 0)

def generate_id():
    return ''.join(random.choice(string.ascii_lowercase) for _ in range(5))

def setStartTimes(groupState):
    # Takes simCF and simTimeHelper and adds start time to cf commands
    simTimeHelper = groupState.timeHelper
    simCFs = groupState.crazyflies

    assert([len(cf.commands) == len(simTimeHelper.times) for cf in simCFs])

    for cf in simCFs:
        for (command, time) in zip(cf.commands, simTimeHelper.times):
            command.startTime = time

class CrazyflieSimLogger:
    
    def __init__(self, initial_position):
        self.node = Node('logger' + generate_id())
        self.commands = []
        self.initialPosition = initial_position
        self.currPosition = initial_position

    def goTo(self, pos, yaw, duration, relative=False):
        goToCommand = Command(command='goTo', position=pos, yaw=yaw, relative=relative, duration=duration)
        self.commands.append(goToCommand)
        self.currPosition = pos
    
    def cmdPosition(self, pos):
        cmdPosCommand = Command(command='cmdPos', position=pos, duration=1/Hz)
        self.commands.append(cmdPosCommand)
        self.currPosition = pos
    
    def setLEDColor(self, r, g, b):
        colorCommand = Command(command='setLEDColor', color=(r, g, b))
        self.currPosition.append(colorCommand)
    
    def takeoff(self, height, duration):
        takeoffCommand = Command(command='takeoff', height=height, duration=duration)
        self.commands.append(takeoffCommand)
        self.currPosition[2] = height # Just set Z to height

    def land(self, height, duration):
        landCommand = Command(command='land', height=height, duration=duration)
        self.commands.append(landCommand)
        self.currPosition[2] = height

    def position(self):
        return self.currPosition

    def notifySetpointsStop(self):
        # Just check mode in Evaluate phase
        self.commands.append(Command(command='notifySetpointsStop'))

class TimeHelperSimLogger:
    def __init__(self):
        self.times = []
        self.currTime = 0

    def time(self):
        return self.currTime

    def sleep(self, duration):
        self.times.append(self.currTime)
        self.currTime += duration
    
    def sleepForRate(self, Hz):
        self.times.append(self.currTime)
        self.currTime += 1/Hz
    
    def sleepUntil(self, time):
        self.times.append(self.currTime)
        self.currTime = time