/* eslint-disable @typescript-eslint/no-unused-vars */
/* eslint-disable @typescript-eslint/no-unsafe-call */
import {type RobartBlockDefinition} from '../BlockDefinition';
import Blockly from 'blockly';
import * as SIM from '@MRAControl/state/simulatorCommands';
import {Simulation} from '@MRAControl/layout/simulation/Simulation';

export const blockSubtractTrajectories: RobartBlockDefinition = {
	name: 'subtractTrajectories',
	block: {
		init: function () {
			this.appendDummyInput()
				.appendField('Subtract Trajectories');
			this.appendStatementInput('firstTrajectory')
				.appendField('First Trajectory:');
			this.appendStatementInput('secondTrajectory')
				.appendField('Second Trajectory:');
			this.setPreviousStatement(true, null);
			this.setNextStatement(true, null);
			this.setColour(90);
			this.setTooltip('Subtract two trajectories (see add).');
			this.setHelpUrl('');
		},
	},
	pythonGenerator: (block, python) => {
		var firstTrajectory = python.statementToCode(block, 'firstTrajectory');
		var secondTrajectory = python.statementToCode(block, 'secondTrajectory');
		firstTrajectory = firstTrajectory.trim();
		secondTrajectory = secondTrajectory.trim();

		return `subtractTrajectories(groupState, lambda groupState: ${firstTrajectory}, lambda groupState: ${secondTrajectory}, False)\n`;
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

		return `simulator.addTrajectories(groupState, ${firstTrajectory}, ${secondTrajectory}, false)\n`;
	},
};