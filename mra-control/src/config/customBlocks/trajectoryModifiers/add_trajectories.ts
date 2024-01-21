/* eslint-disable @typescript-eslint/no-unused-vars */
/* eslint-disable @typescript-eslint/no-unsafe-call */
import {type RobartBlockDefinition} from '../BlockDefinition';
import Blockly from 'blockly';
import * as SIM from '@MRAControl/state/simulatorCommands';
import {Simulation} from '@MRAControl/layout/simulation/Simulation';

export const blockAddTrajectories: RobartBlockDefinition = {
	name: 'addTrajectories',
	block: {
		init: function () {
			this.appendDummyInput()
				.appendField('Add Trajectories');
			this.appendStatementInput('firstTrajectory')
				.appendField('First Trajectory:');
			this.appendStatementInput('secondTrajectory')
				.appendField('Second Trajectory:');
			this.setPreviousStatement(true, null);
			this.setNextStatement(true, null);
			this.setColour(90);
			this.setTooltip('Add two trajectories together. At every timestep, the output (relative change from initial position) of each trajectory is added together. When trajectories have different durations, this will run for the longer of the two durations. Note: Cannot take multiple vertically stacked blocks as input.');
			this.setHelpUrl('');
		},
	},
	pythonGenerator: (block, python) => {
		var firstTrajectory = python.statementToCode(block, 'firstTrajectory');
		var secondTrajectory = python.statementToCode(block, 'secondTrajectory');
		firstTrajectory = firstTrajectory.trim();
		secondTrajectory = secondTrajectory.trim();
		const code = `addTrajectories(groupState, lambda groupState: ${firstTrajectory}, lambda groupState: ${secondTrajectory}\n`;
		return code;
	},
	javascriptGenerator: (block, js) => {
		// var xPosition = block.getFieldValue('x_pos') as number;
		// var yPosition = block.getFieldValue('y_pos') as number;
		// var zPosition = block.getFieldValue('z_pos') as number;
		// var duration = block.getFieldValue('duration') as number;
		var firstTrajectory = js.statementToCode(block, 'firstTrajectory');
		var secondTrajectory = js.statementToCode(block, 'secondTrajectory');
		firstTrajectory = firstTrajectory.trim();
		secondTrajectory = secondTrajectory.trim();

		return `simulator.addTrajectories(groupState, ${firstTrajectory}, ${secondTrajectory})\n`;
	},
};