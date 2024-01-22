/* eslint-disable @typescript-eslint/no-unused-vars */
/* eslint-disable @typescript-eslint/no-unsafe-call */
import {type RobartBlockDefinition} from '../BlockDefinition';
import Blockly from 'blockly';

import * as SIM from '@MRAControl/state/simulatorCommands';
export const blockMoveXyz: RobartBlockDefinition = {
	name: 'move_xyz',
	block: {
		init: function () {
			this.appendDummyInput()
				.appendField('move X')
				.appendField(new Blockly.FieldNumber(0), 'x')
				.appendField('meters');
			this.appendDummyInput()
				.appendField('move Y')
				.appendField(new Blockly.FieldNumber(0), 'y')
				.appendField('meters');
			this.appendDummyInput()
				.appendField('move Z')
				.appendField(new Blockly.FieldNumber(0), 'z')
				.appendField('meters');
			this.appendDummyInput()
				.appendField('at')
				.appendField(new Blockly.FieldNumber(0.5, 0.01), 'speed')
				.appendField('m/s');
			this.setPreviousStatement(true, null);
			this.setNextStatement(true, null);
			this.setColour(90);
			this.setTooltip('Move a direction at an average given speed.');
			this.setHelpUrl('');
		},
	},
	pythonGenerator: (block, python) => {
		var x = block.getFieldValue('x') as number;
		var y = block.getFieldValue('y') as number;
		var z = block.getFieldValue('z') as number;
		var speed = block.getFieldValue('speed') as number;
		var code = 'goto_velocity_relative_position(groupState, ' + x + ',' + y + ',' + z + ',' + speed + ')\n';
		return code;
	},
	javascriptGenerator: (block, _js) => {
		var x = block.getFieldValue('x') as number;
		var y = block.getFieldValue('y') as number;
		var z = block.getFieldValue('z') as number;
		var speed = block.getFieldValue('speed') as number;
		return `simulator.moveSpeed(groupState, ${x}, ${y}, ${z}, ${speed})\n`;
	},
};