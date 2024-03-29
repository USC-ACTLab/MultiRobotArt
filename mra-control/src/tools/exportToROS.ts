/* eslint-disable @typescript-eslint/no-for-in-array */
import {RobotState, type MRAState} from '@MRAControl/state/useRobartState';
import JsZip from 'jszip';
import FileSaver from 'file-saver';

// eslint-disable-next-line import/no-webpack-loader-syntax
import workerNodeTemplate from './python-templates/worker_node.py?raw';
import launcherNode from './python-templates/launch_block_nodes.py?raw';
import configure from './python-templates/configure.py?raw';
import translations from './python-templates/blocklyTranslations.py?raw';
import launchServer from './python-templates/launch.py?raw';
import timeHelper from './python-templates/TimeHelper.py?raw';
import crazyflieLoggers from './python-templates/crazyflieLoggers.py?raw';
import readmeText from './python-templates/Readme.md?raw';

const extractLineNumber = (text: string, toMatch: string): number => {
	// Get index of first match, split by lines and count # lines until occurance
	const index = text.indexOf(toMatch);
	return text.substring(0, index).split('\n').length;
};

/**
 * Export project to ROS code
 * @param projectState The name of the file to download
 * @param filename
 */
// eslint-disable-next-line @typescript-eslint/naming-convention
export const exportToROS = async (projectState: MRAState, fileName: string) => {
	const execBlocksLine = extractLineNumber(workerNodeTemplate, '---------Insert Execution Code Here------------');
	let launcherNodeLine = extractLineNumber(launcherNode, '-----------Insert Nodes Here-----------');
	var mainNode = launcherNode.split('\n');
	var numGroups = 0; //Object.keys(projectState.timelineState.groups).length;
	const zip = JsZip();
    const allRobots = projectState.robots
    const robotIndexMap: { [id: string]: number } = {};

    Object.keys(allRobots).forEach((id: string, index: number) => {
        robotIndexMap[id] = index;
    });

	const groups = projectState.timelineState.groups;
	for (let groupName in groups) {
		const groupState = projectState.timelineState.groups[groupName];
		const robots = Object.values(groupState.robots);

		const robotIndices: number[] = [];
		if (robots.length === 0) {
			continue;
		}

		numGroups += 1;
		for (let r in robots) {
            const state = robots[r]
            const index = robotIndexMap[state.id];

            // const r = groupState.robots[robotId];
			// console.log(robots, r);
			// if (robots[r] == undefined) {
			// 	continue;
			// }

			robotIndices.push(index);
		}

		var pythonBlocks = '';
		if (Object.keys(groupState.items).length === 0) {
			continue;
		}
        const sortedItems = Object.entries(groupState.items).sort((a, b) => {
            return a[1].startTime - b[1].startTime
        })
        

		for (let [block, blockState] of sortedItems) {
			const startTime = blockState.startTime;
			const pythonCode = projectState.blocks[blockState.blockId].python;
            pythonBlocks += '        # Block Name: ' + projectState.blocks[blockState.blockId].name + '\n';
			pythonBlocks += `        start_time = ${startTime}\n`;
			pythonBlocks += '        self.timeHelper.sleepUntil(start_time)\n';
                
			var pythonLines = pythonCode.split('\n');
			for (var i = 0; i < pythonLines.length - 1; i++) {
				pythonBlocks += `        ${pythonLines[i]}\n`;
			}
			// }
		}

		// Read in template
		var workerNode = workerNodeTemplate.split('\n');

		// Inject trajectories into template
		// workerNode.splice(trajLine, 0, pythonTrajectories);
		workerNode.splice(execBlocksLine, 0, pythonBlocks);

		// For each worker, save file and insert start code to main node. 
		var launcherText = `    import ${groupName}` + '_node\n';
		launcherText += '    cfs = []\n';

		for (const r of robotIndices) {
            if (r === undefined){
                continue;
            }
			launcherText += `    cfs.append(crazyflies[${r}])\n`;
        }
		launcherText += `    nodes.append(${groupName}_node.worker_node(cfs, len(nodes), ${numGroups}))\n`;
        launcherText += `\n`;
		mainNode.splice(launcherNodeLine, 0, launcherText);
		launcherNodeLine += 1;

		// Save worker node
		const workerNodeFile = new Blob([workerNode.join('\n')], {type: 'text/plain'});
		zip.file(`${groupName}_node.py`, workerNodeFile);
	}

	//const element = document.createElement('a');
	const launcherFile = new Blob([mainNode.join('\n')], {type: 'text/plain'});

	// Configuration file
	const configureFile = new Blob([configure], {type: 'text/plain'});
    const loggersFile = new Blob([crazyflieLoggers], {type: 'text/plain'});
    const readmeFile = new Blob([readmeText], {type: 'text/plain'});

	// Starting positions yaml
	var startingPositions = 'positions:\n';
	for (const robot in projectState.robots) {
		var pos = [0, 0, 0];
		if (projectState.robots[robot] != undefined)
			pos = projectState.robots[robot].startingPosition;
		startingPositions += `    - [${pos[0]}, ${pos[1]}, ${pos[2]}]\n`;
	}

	const startingPosFile = new Blob([startingPositions], {type: 'text/plain'});


	zip.file('starting_positions.yaml', startingPosFile);
	zip.file('configure.py', configureFile);
	zip.file('launch_nodes.py', launcherFile);
    zip.file('TimeHelper.py', timeHelper);
    zip.file('crazyflieLoggers.py', loggersFile);
    zip.file('Readme.md', readmeFile);

    
    const translationsFile = new Blob([translations], {type: 'text/plain'});
    zip.file('blocklyTranslations.py', translationsFile);

    const launchFile = new Blob([launchServer], {type: 'text/plain'});
    zip.file('launch.py', launchFile)

	const zipFile = await zip.generateAsync({type: 'blob'});
	FileSaver.saveAs(zipFile, `${projectState.projectName}.zip`);
};