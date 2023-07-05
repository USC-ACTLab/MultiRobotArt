import { MRAState } from '@MRAControl/state/useRobartState';
import { open } from 'node:fs/promises';
// import * as fs from 'fs';
import fs from 'fs';
import { config } from 'node:process';
// TODO: Make these not hard coded...
const trajLine = 45;
const execBlocksLine = 88;
var launcherNodeLine = 39

/**
 * Export project to ROS code
 * @param projectState The name of the file to download
 * @param filename
 */
export const exportToROS = async (projectState: MRAState ,fileName: string) => {
    var mainNode = launcherNode.split('\n');
    var numGroups = 0; //Object.keys(projectState.timelineState.groups).length;
    var trajCounter = 0
    const groups = projectState.timelineState.groups;
    for (let groupName in groups){
        const groupState = projectState.timelineState.groups[groupName];
        const robots = Object.values(groupState.robots);
        var robotIndices: string[] = [];
        if (robots.length === 0){
            continue
        }
        numGroups += 1;
        for (let r in robots){
            console.log(robots, r);
            if (robots[r] == undefined){
                continue
            }
            robotIndices.push(r);
            console.log("Robot Indices", robotIndices)
        }
        var pythonTrajectories = "";

        var pythonBlocks = "";
        if (Object.keys(groupState.items).length === 0){
            console.log("No items" + groupState)
            continue;
        }
        else{
            console.log(groupState.items)
        }
        console.log("Items: " + groupState.items)
        for (let block in groupState.items){
            console.log(block)
            const blockState = groupState.items[block];
            const startTime = blockState.startTime;
            const pythonCode = projectState.blocks[blockState.blockId].python;
            if (blockState.isTrajectory){
                pythonTrajectories += '        trajectories.append(' + pythonCode + ')';
                pythonBlocks += '        start_time = ${startTime}\n';
                pythonBlocks += '        self.wait_until(start_time)\n';
                pythonBlocks += '        for cf in self.crazyflies:\n';
                pythonBlocks += '            cf.startTrajectory(${trajCounter}, 0, 1)\n';
                pythonBlocks += '        self.wait_until(start_time + self.trajectories[${trajCounter}].duration\n';
                trajCounter += 1
            }
            else{
                const duration = 1.0; //TODO figure out how to get duration
                pythonBlocks += `        start_time = ${startTime}\n`;
                pythonBlocks += `        total_duration = ${duration}\n`;
                pythonBlocks += `        self.wait_until(start_time)\n`;
                
                var pythonLines = pythonCode.split('\n');
                // TODO: Is this -1 needed or just a bug?
                for (var i = 0; i < pythonLines.length - 1; i++){
                    pythonBlocks += `        for cf in self.crazyflies:\n`;
                    pythonBlocks += `            block_duration = ${pythonLines[i]}\n`;
                    pythonBlocks += '        self.wait(block_duration)\n'
                }
                pythonBlocks += `        self.wait_until(start_time + total_duration)\n`;
            }
        }
        // Read in template
        var workerNode = workerNodeTemplate.split('\n');

        // Inject trajectories into template
        workerNode.splice(trajLine, 0, pythonTrajectories);
        workerNode.splice(execBlocksLine, 0, pythonBlocks);

        // For each worker, save file and insert start code to main node. 
        console.log(robotIndices);
        var launcher_text = `    import ${groupName}` + `_node\n`;
        launcher_text = `    cfs = []\n`

        for (let r in robotIndices) 
            launcher_text += `    cfs.append(crazyflies[${r}])\n`;
        launcher_text += `    nodes.append(${groupName}_node.worker_node(cfs, len(nodes)-1, ${numGroups}))\n`;
        mainNode.splice(launcherNodeLine, 0, launcher_text);
        launcherNodeLine += 2;

        // Save worker node
        // fs.writeFile(groupName + '_node.py', workerNode.join('\n'), function (err) {
        //     if (err) return console.log(err);
        // });
        const element = document.createElement('a');
        const worker_node = new Blob([workerNode.join('\n')], { type: 'text/plain' });
        element.href = URL.createObjectURL(worker_node);
        element.download = `${groupName}_node.py`;
        document.body.appendChild(element); // Required for this to work in FireFox
        element.click();
        document.body.removeChild(element); // Clean up after ourselves.
    }
    const element = document.createElement('a');
    const launcherFile = new Blob([mainNode.join('\n')], { type: 'text/plain' });
    element.href = URL.createObjectURL(launcherFile);
    element.download = 'launch_nodes.py';
    document.body.appendChild(element); // Required for this to work in FireFox
    element.click();
    document.body.removeChild(element); // Clean up after ourselves.

    // Configuration file
    const configureFile = new Blob([configure], { type: 'text/plain' });
    element.href = URL.createObjectURL(configureFile);
    element.download = 'configure.py';
    document.body.appendChild(element); // Required for this to work in FireFox
    element.click();
    document.body.removeChild(element); // Clean up after ourselves.

    // Starting positions yaml
    var startingPositions = `positions:\n`;
    for(let robot in Object.values(projectState.robots)){
        var pos = [0, 0, 0]
        if(projectState.robots[robot] != undefined)
            pos = projectState.robots[robot].startingPosition;
            // TODO All 0s?
            startingPositions += `    - [${pos[0]}, ${pos[1]}, ${pos[2]}]\n`;
    }

    const startingPosFile = new Blob([startingPositions], { type: 'text/plain' });
    element.href = URL.createObjectURL(startingPosFile);
    element.download = 'starting_positions.yaml';
    document.body.appendChild(element); // Required for this to work in FireFox
    element.click();
    document.body.removeChild(element); // Clean up after ourselves.
  };

const workerNodeTemplate =  
`import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from crazyflie_py import generate_trajectory
import numpy as np
from blocklyTranslations import *

Hz = 30

class worker_node(Node):

    def __init__(self, crazyflies, id=0, num_nodes=1):
        """
        id: a unique id between 0 and num_nodes corresponding to the thread number of this worker
        num_nodes: number of nodes (threads) in total
        """
        super().__init__("worker_node_{}".format(id))
        self.id = id
        self.num_nodes = num_nodes
        self.crazyflies = crazyflies

        self.execution_ready_subscription(
            Int32,
            'ready',
            self.ready_callback,
            num_nodes + 1
        )
        self.execution_ready_publisher(
            Int32,
            'ready',
            num_nodes + 1
        )
        self.timer = self.create_timer(1/Hz, self.timer_callback)
        self.ready_ids = set()
        self.executing = False
        self.running = False
        self.done = False
    
    def compute_trajectories(self):
        """
        Inject Trajectory computation code here...
        """
        trajectories = []
        
        #TODO insert trajectories here...

        return trajectories

    def upload_trajectories(crazyflies, trajectories):
        '''
            Upload trajectories to crazyflies one by one
        '''

        # TODO: Currently doesn't support overlapping crazyflies as we will overwrite trajectories...
        for i, traj in enumerate(trajectories):
            for cf in crazyflies:
                cf.uploadTrajectory(traj, i, 0)

    def begin(self):
        """
            Prepare for execution. Pre-compute trajectories and upload them to crazyflies
        """
        trajectories = self.compute_trajectories()
        self.upload_trajectories(trajectories)
        msg = Int32()
        msg.data = self.id
        self.ready_publisher.publish(msg)

    def time(self):
        return self.get_clock().now().nanoseconds / 1e9
    
    def execute_blocks(self):
        """
        Must be injected into...

        Typical format should be:
        
        start_time = 0.0
        duration = 3.0
        self.wait_until(start_time)
        for cf in self.crazyflies:
            cf.takeoff(1.0, duration)
        self.wait_until(start_time+duration)

        where start_time, duration, and in the inner portion of the for loop are provided.
        """
        # BLOCKS...
        pass

    def ready_callback(self, msg):
        self.ready_ids.add(msg.data)
    
    def timer_callback(self):
        if not self.running:
            self.begin()
            self.running = True
        if len(self.ready_ids) == self.num_nodes and not self.executing:
            self.running = True
            self.start_time = self.get_clock().now()
            self.execute_blocks()
            self.destroy_node()
            self.done = True

    def wait_until(self, end_time):
        while self.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0)

    def wait(self, time):
        end_time = self.time() + time
        self.wait_until(end_time)
`;

const launcherNode = 
`import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from crazyflie_py import Crazyswarm
import rclpy
import threading

# Inject Imports Here:

def launch(nodes):
    threads = []
    for node in nodes:
        thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
        thread.start()
        threads.append(thread)
    while not all([n.done for n in nodes]):
        pass
    rclpy.shutdown()
    [t.join() for t in threads]

def main():
    # node_config_file = get_package_share_directory('crazyflie'), 'config', 'crazyflies.yaml'
    
    # Initialize swarm
    swarm = Crazyswarm()
    with open("cfs_ordering.yaml") as f:
        ordering = yaml.load(f)
        order = ordering['cfs']
    # all_crazyflies = swarm.allcfs.crazyflies
    all_crazyflies = [swarm.allcfs.crazyfliesById[k] for k in order]
    # Construct Launch Description from yaml files
    # One node per line in the timeline, linked to a worker node described in the yaml file
    counter = 0
    nodes = []
    cfs = []

    # Inject append here...

    # cfs.append(all_crazyflies[0])
    # nodes.append(group1_node.worker_node(cfs, len(cfs), 1))
    
    # Launch all nodes
    return launch(nodes)

if __name__ == '__main__':
    main()
`;

const configure = 
`# Need to ask user which crazyflies they will use, and will tell them where to place them

import yaml
import copy
import os, sys, stat

# Load starting positions from blockly
with open('starting_positions.yaml') as f:
    starting_positions = yaml.load(f)

n_robots = len(starting_positions['positions'])

print("You will need {} crazyflies".format(n_robots))
print("Which crazyflies (by ID) will you use?")

cfs = []
for i in range(n_robots):
    cf = input("crazyflie #{}: ".format(i + 1))
    cfs.append(cf)

    
start_dict = {}
for cf, pos in zip(cfs, starting_positions['positions']):
    print("Place cf {} at location {}".format(cf, pos))
    start_dict[cf] = pos

# Make changes to local mycrazyflies.yaml
with open('crazyflies.yaml') as f:
    all_crazyflies = yaml.load(f)

my_cfs = copy.deepcopy(all_crazyflies)
for r in all_crazyflies['robots']:
    id = r.strip('cf')
    if id not in cfs:
        del my_cfs['robots'][r]
    else:
        my_cfs['robots'][r]['enabled'] = True
        my_cfs['robots'][r]['initial_position'] = start_dict[id]


with open('my_crazyflies.yaml', 'w') as f:
    yaml.dump(my_cfs, f)

# Make commands for sim and non-sim launches
with open('sim.sh', 'w') as f:
    f.write('ros2 launch launch.py config:=my_crazyflies.yaml backend:=sim')
    os.chmod('sim.sh', stat.S_IRWXU)
with open('run.sh', 'w') as f:
    f.write('ros2 launch launch.py config:=my_crazyflies.yaml')
    os.chmod('run.sh', stat.S_IRWXU)
`

const translations = 
`import numpy as np
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
`;