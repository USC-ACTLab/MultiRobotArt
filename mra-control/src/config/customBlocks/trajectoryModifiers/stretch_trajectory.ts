/* eslint-disable @typescript-eslint/no-unused-vars */
/* eslint-disable @typescript-eslint/no-unsafe-call */
import {type RobartBlockDefinition} from '../BlockDefinition';
import Blockly from 'blockly';
import * as SIM from '@MRAControl/state/simulatorCommands';
import {Simulation} from '@MRAControl/layout/simulation/Simulation';

export const blockStretchTrajectories: RobartBlockDefinition = {
	name: 'stretch',
	block: {
		init: function () {
			this.appendDummyInput()
				.appendField('Stretch Trajectory')
				.appendField('x: ')
				.appendField(new Blockly.FieldNumber(1), 'x')
				.appendField(' y: ')
				.appendField(new Blockly.FieldNumber(1), 'y')
				.appendField(' z: ')
				.appendField(new Blockly.FieldNumber(1), 'z')
				.appendField(' time: ')
				.appendField(new Blockly.FieldNumber(1), 't');
			this.appendStatementInput('originalTraj')
				.appendField('Trajectory: ');
			this.setPreviousStatement(true, null);
			this.setNextStatement(true, null);
			this.setColour(90);
			this.setTooltip('');
			this.setHelpUrl('');
		},
	},
	pythonGenerator: (block, _python) => {
		// TODO...
		let code = 'pass';
		return code;
	},
	javascriptGenerator: (block, js) => {
		var originalTraj = js.statementToCode(block, 'originalTraj');
		originalTraj = originalTraj.trim();
		const x = block.getFieldValue('x') as number;
		const y = block.getFieldValue('y') as number;
		const z = block.getFieldValue('z') as number;
		const t = block.getFieldValue('t') as number;


		return `simulator.stretchTrajectory(groupState, ${originalTraj}, ${x}, ${y}, ${z}, ${t})\n`;
	},
};