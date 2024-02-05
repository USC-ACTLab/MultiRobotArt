
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
    """A Data class to contain information about commands.
    Used by simulated crazyflies to keep track of simulated commands
    """    
    command: Any = None
    startTime: float = 0.0
    position: Collection = (0, 0, 0)
    yaw: float = 0
    duration: float = 0
    relative: bool = False
    height: float = 0
    color: Collection = (0, 0, 0)

def generate_id():
    """Generate a random ID number for unique node names

    Returns:
        string: unique (probably) id
    """    
    return ''.join(random.choice(string.ascii_lowercase) for _ in range(5))

def setStartTimes(groupState):
    """Set the appropriate start times for every command a simulated crazyflie ran (with a simulated timeHelper)

    Args:
        groupState : current (simulated) groupState (crazyflies and timehelper)
    """    
    # Takes simCF and simTimeHelper and adds start time to cf commands
    simTimeHelper = groupState.timeHelper
    simCFs = groupState.crazyflies

    assert([len(cf.commands) == len(simTimeHelper.times) for cf in simCFs])

    for cf in simCFs:
        for (command, time) in zip(cf.commands, simTimeHelper.times):
            command.startTime = time

class CrazyflieSimLogger:
    """ 
    Crazyflie Logging class used instead of a standard Crazyflie object
    when commands should *not* be executed. For example, when all commands
    will be rotated or negated.
    """    
    
    def __init__(self, initial_position):
        """
        Args:
            initial_position [float, float ,float]: starting position of Crazyflie at time of construction
        """        
        self.node = Node('logger' + generate_id())
        self.commands = []
        self.initialPosition = initial_position
        self.currPosition = initial_position

    def goTo(self, pos, yaw, duration, relative=False):
        """Simulate a goTo command

        Args:
            pos [float, float, float]: Desired destination of command
            yaw float: Desired yaw
            duration float: length of time of command
            relative (bool, optional): Whether pos and yaw are relative to the starting position. Defaults to False.
        """
        goToCommand = Command(command='goTo', position=pos, yaw=yaw, relative=relative, duration=duration)
        self.commands.append(goToCommand)
        self.currPosition = pos
    
    def cmdPosition(self, pos):
        """Simulate cmdPos (low level command)

        Args:
            pos [float, float, float]: desired position
        """        
        cmdPosCommand = Command(command='cmdPos', position=pos, duration=1/Hz)
        self.commands.append(cmdPosCommand)
        self.currPosition = pos
    
    def setLEDColor(self, r, g, b):
        """Simulate setLEDColor 
        If all parameters are <1, then float representations are assumed (color is given in the range [0, 1])
        If not, then an 8 bit representation is assumed (color is given in the range [0, 255])

        Args:
            r float: red
            g float: green
            b float: blue
        """        
        colorCommand = Command(command='setLEDColor', color=(r, g, b))
        self.currPosition.append(colorCommand)
    
    def takeoff(self, height, duration):
        """Simulate takeoff command

        Args:
            height float: Desired height
            duration float: duration of command
        """        
        takeoffCommand = Command(command='takeoff', height=height, duration=duration)
        self.commands.append(takeoffCommand)
        self.currPosition[2] = height # Just set Z to height

    def land(self, height, duration):
        """Simulate land command

        Args:
            height float: Desired height
            duration float: duration of command
        """
        landCommand = Command(command='land', height=height, duration=duration)
        self.commands.append(landCommand)
        self.currPosition[2] = height

    def position(self):
        """get Position

        Returns:
            [float, float, float]: current position
        """        
        return self.currPosition

    def notifySetpointsStop(self):
        """Switch to high level mode
        """        
        # Just check mode in Evaluate phase
        self.commands.append(Command(command='notifySetpointsStop'))

class TimeHelperSimLogger:
    """A simulated timer class to go with CrazyflieSimLogger
    Does not actually sleep, but logs sleep durations to be used by a non-sim time helper
    """    
    def __init__(self):
        self.times = []
        self.currTime = 0

    def time(self):
        """Get current time of timehelper (in seconds)

        Returns:
            float: current time in seconds
        """        
        return self.currTime

    def sleep(self, duration):
        """sleep for a given duration (simulated)

        Args:
            duration float: duration of sleep
        """        
        self.times.append(self.currTime)
        self.currTime += duration
    
    def sleepForRate(self, Hz):
        """Simulate sleep for a specified rate (sleep for 1/Hz)

        Args:
            Hz float: rate to sleep at
        """        
        self.times.append(self.currTime)
        self.currTime += 1/Hz
    
    def sleepUntil(self, time):
        """Sleep until a specified time (in seconds)

        Args:
            time float: time to sleep until (in seconds)
        """        
        self.times.append(self.currTime)
        self.currTime = time