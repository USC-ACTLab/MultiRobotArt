/* eslint-disable @typescript-eslint/no-unused-vars */
/* eslint-disable @typescript-eslint/no-unsafe-call */
import {type RobartBlockDefinition} from '../BlockDefinition';
import Blockly from 'blockly';
import * as SIM from '@MRAControl/state/simulatorCommands';
import {Simulation} from '@MRAControl/layout/simulation/Simulation';

export const blockRotateTrajectoryDegrees: RobartBlockDefinition = {
	name: 'rotateDegrees',
	block: {
		init: function () {
			this.appendDummyInput()
				.appendField('Rotate Trajectory (degrees)')
				.appendField('x: ')
				.appendField(new Blockly.FieldNumber(0), 'x')
				.appendField(' y: ')
				.appendField(new Blockly.FieldNumber(0), 'y')
				.appendField(' z: ')
				.appendField(new Blockly.FieldNumber(0), 'z');
			this.appendStatementInput('originalTraj')
				.appendField('Trajectory: ');
			this.setPreviousStatement(true, null);
			this.setNextStatement(true, null);
			this.setColour(90);
			this.setTooltip('Rotate the given trajectory by the given Euler Angles.');
			this.setHelpUrl('');
		},
	},
	pythonGenerator: (block, python) => {
		var originalTraj = python.statementToCode(block, 'originalTraj');
		originalTraj = originalTraj.trim();
		const x = block.getFieldValue('x') as number * Math.PI / 180;
		const y = block.getFieldValue('y') as number * Math.PI / 180;
		const z = block.getFieldValue('z') as number * Math.PI / 180;
		let code = `rotate(groupState, lambda groupState ${originalTraj}, ${x}, ${y}, ${z})\n`;
		return code;
	},
	javascriptGenerator: (block, js) => {
		var originalTraj = js.statementToCode(block, 'originalTraj');
		originalTraj = originalTraj.trim();
		const x = block.getFieldValue('x') as number * Math.PI / 180;
		const y = block.getFieldValue('y') as number * Math.PI / 180;
		const z = block.getFieldValue('z') as number * Math.PI / 180;

		return `simulator.rotateTrajectory(groupState, ${originalTraj}, ${x}, ${y}, ${z})\n`;
	},
};
export const blockRotateTrajectoryRadians: RobartBlockDefinition = {
	name: 'rotateRadians',
	block: {
		init: function () {
			this.appendDummyInput()
				.appendField('Rotate Trajectory (radians)')
				.appendField('x: ')
				.appendField(new Blockly.FieldNumber(0), 'x')
				.appendField(' y: ')
				.appendField(new Blockly.FieldNumber(0), 'y')
				.appendField(' z: ')
				.appendField(new Blockly.FieldNumber(0), 'z');
			this.appendStatementInput('originalTraj')
				.appendField('Trajectory: ');
			this.setPreviousStatement(true, null);
			this.setNextStatement(true, null);
			this.setColour(90);
			this.setTooltip('Rotate the given trajectory by the given Euler Angles.');
			this.setHelpUrl('');
		},
	},
	pythonGenerator: (block, python) => {
		var originalTraj = python.statementToCode(block, 'originalTraj');
		originalTraj = originalTraj.trim();
		const x = block.getFieldValue('x') as number;
		const y = block.getFieldValue('y') as number;
		const z = block.getFieldValue('z') as number;
		let code = `rotate(groupState, lambda groupState ${originalTraj}, ${x}, ${y}, ${z})\n`;
		return code;
	},
	javascriptGenerator: (block, js) => {
		var originalTraj = js.statementToCode(block, 'originalTraj');
		originalTraj = originalTraj.trim();
		const x = block.getFieldValue('x') as number;
		const y = block.getFieldValue('y') as number;
		const z = block.getFieldValue('z') as number;

		return `simulator.rotateTrajectory(groupState, ${originalTraj}, ${x}, ${y}, ${z})\n`;
	},
};