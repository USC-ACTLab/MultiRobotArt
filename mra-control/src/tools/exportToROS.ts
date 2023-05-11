import { MRAState } from '@MRAControl/state/useRobartState';
import { open } from 'node:fs/promises';
import * as fs from 'fs';

// TODO: Make these not hard coded...
const trajLine = 42;
const execBlocksLine = 80;
var launcherNodeLine = 27

/**
 * Export project to ROS code
 * @param fileName The name of the file to download
 * @param contents
 */
export const exportToROS = async (projectState: MRAState ,fileName: string) => {
    const element = document.createElement('a');

    var mainNode = fs.readFileSync('./python-templates/launch_block_nodes.py').toString().split('\n');
    const numGroups = Object.keys(projectState.timelineState).length;
    for (let groupName in Object.keys(projectState.timelineState)){
        const groupState = Object.values(projectState.timelineState)[groupName];
        
        // Read in template
        var workerNode = fs.readFileSync('./python-templates/worker_node.py').toString().split('\n');

        // TODO: Get python Trajectories python code
        var pythonTrajectories = "temp";
        
        // TODO: Get block exec code
        var pythonBlocks = "temp";

        // Inject trajectories into template
        workerNode.splice(trajLine, 0, pythonTrajectories);
        workerNode.splice(execBlocksLine, 0, pythonBlocks);

        // For each worker, save file and insert start code to main node. 
        const launcher_text = '    import ${groupName + \'_node\'}\n    description.append(${groupName + \'_node\'}.worker_node(all_crazyflies, counter, ${numGroups}))';
        mainNode.splice(launcherNodeLine, 0, launcher_text);
        launcherNodeLine += 2;

        // Save worker node
        fs.writeFile(groupName + '_node.py', workerNode.join('\n'), function (err) {
            if (err) return console.log(err);
        });
    }
    
    const file = new Blob([contents], { type: 'text/plain' });

    element.href = URL.createObjectURL(file);
    element.download = fileName;
    document.body.appendChild(element); // Required for this to work in FireFox
    element.click();
    document.body.removeChild(element); // Clean up after ourselves.
  };
  