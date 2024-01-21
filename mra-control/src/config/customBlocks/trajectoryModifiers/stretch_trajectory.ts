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
				.appendField(new Blockly.FieldNumber(1, 0.0), 'x')
				.appendField(' y: ')
				.appendField(new Blockly.FieldNumber(1, 0.0), 'y')
				.appendField(' z: ')
				.appendField(new Blockly.FieldNumber(1, 0.0), 'z')
				.appendField(' time: ')
				.appendField(new Blockly.FieldNumber(1, 0.0), 't');
			this.appendStatementInput('originalTraj')
				.appendField('Trajectory: ');
			this.setPreviousStatement(true, null);
			this.setNextStatement(true, null);
			this.setColour(90);
			this.setTooltip('Stretch or shrink a trajectory by any component or time. A stretch of 0.5 will make all x values in the trajectory shrink by half, a stretch of 2.0 will double all x values in the trajectory. A time stretch factor of 0.5 will make the trajectory take half the time (twice as fast), and a factor of 2.0 will make it take twice as long (half as fast).');
			this.setHelpUrl('');
		},
	},
	pythonGenerator: (block, python) => {
		var originalTraj = python.statementToCode(block, 'originalTraj');
		originalTraj = originalTraj.trim();
		const x = block.getFieldValue('x') as number;
		const y = block.getFieldValue('y') as number;
		const z = block.getFieldValue('z') as number;
		const t = block.getFieldValue('t') as number;


		return `stretchTrajectory(groupState, lambda groupState: ${originalTraj}, ${x}, ${y}, ${z}, ${t})\n`;
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