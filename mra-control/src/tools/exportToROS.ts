import { MRAState } from '@MRAControl/state/useRobartState';
import { open } from 'node:fs/promises';
// import * as fs from 'fs';
import fs from 'fs';
// TODO: Make these not hard coded...
const trajLine = 42;
const execBlocksLine = 80;
var launcherNodeLine = 27

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
        // TODO Assumes CFs are named with indices
        for (let r in robotIndices) 
            launcher_text += `    cfs.append(crazyflies[${r}])\n`;
        launcher_text += `    description.append(${groupName}_node.worker_node(cfs, counter, ${numGroups}))\n`;
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
  };

const workerNodeTemplate =  `import rclpy
from rclpy.node import Node
from std_msgs.msg import int32
from crazyflie_py import generate_trajectory
import numpy as np

Hz = 100

class worker_node(Node):

    def __init__(self, crazyflies, id=0, num_nodes=1):
        """
        id: a unique id between 0 and num_nodes corresponding to the thread number of this worker
        num_nodes: number of nodes (threads) in total
        """
        self.id = id
        self.num_nodes = num_nodes
        self.crazyflies = crazyflies

        self.execution_ready_subscription(
            int32,
            'ready',
            self.ready_callback,
            num_nodes + 1
        )
        self.execution_ready_publisher(
            int32,
            'ready',
            num_nodes + 1
        )
        self.timer = self.create_timer(1/Hz, self.timer_callback)
        self.ready_ids = set()
        self.executing = False
        self.running = False
    
    def compute_trajectories():
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
        self.execution_ready_publisher.publish(self.id)

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
        self.ready_ids.add(msg)
    
    def timer_callback(self):
        if not self.running:
            self.begin()
            self.running = True
        if len(self.ready_ids) == self.num_nodes and not self.executing:
            self.running = True
            self.start_time = self.node.get_clock().now().time()
            self.execute_blocks()
            self.destory_node()

    def wait_until(self, end_time):
        while self.get_clock().now() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0)
    
    def wait(self, time):
        end_time = self.get_clock.now() + time
        self.wait_until(end_time)
`

const launcherNode = `import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from crazyflie_py import CrazySwarm

#TODO: Inject imports to correctly import nodes
# import test_worker


def generate_launch_description():
    # TODO: Change package name to correct description
    node_config_file = get_package_share_directory('my_package'), 'config', 'node_config.yaml'
    with open(node_config_file, 'r') as f:
        node_configs = yaml.load(f)
    
    # Initialize swarm
    swarm = CrazySwarm()
    all_crazyflies = swarm.allcfs.crazyflies

    # Construct Launch Description from yaml files
    # One node per line in the timeline, linked to a worker node described in the yaml file
    node_configs = node_configs['nodes']
    counter = 0
    description = []
    #TODO Inject append here...
    
    # description.append(test_worker.worker_node(all_crazyflies, counter, len(node_configs)))
    counter += 1
    
    

    # Launch all nodes
    return LaunchDescription(description)`