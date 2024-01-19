/* eslint-disable @typescript-eslint/no-unused-vars */
/* eslint-disable @typescript-eslint/no-unsafe-call */
import {type RobartBlockDefinition} from '../BlockDefinition';
import Blockly from 'blockly';
import * as SIM from '@MRAControl/state/simulatorCommands';

export const blockGoToSpeed: RobartBlockDefinition = {
	name: 'go_to_speed',
	block: {
		init: function () {
			this.appendDummyInput()
				.appendField('go to X:')
				.appendField(new Blockly.FieldNumber(0), 'x_pos')
				.appendField('Y:')
				.appendField(new Blockly.FieldNumber(0), 'y_pos')
				.appendField('Z:')
				.appendField(new Blockly.FieldNumber(0), 'z_pos')
				.appendField('at')
				.appendField(new Blockly.FieldNumber(0.5, 0.01), 'speed')
				.appendField('m/s');
			this.setPreviousStatement(true, null);
			this.setNextStatement(true, null);
			this.setColour(90);
			this.setTooltip('Move to a desired position at a certain speed. NOTE: Desired speed is an average speed, not a maximum speed.');
			this.setHelpUrl('');
		},
	},
	pythonGenerator: (block, _python) => {
		var xPos = block.getFieldValue('x_pos') as number;
		var yPos = block.getFieldValue('y_pos') as number;
		var zPos = block.getFieldValue('z_pos') as number;
		var speed = block.getFieldValue('speed') as number;
		var code = 'go_to(groupState, ' + xPos + ',' + yPos + ',' + zPos + ',' + speed + ')\n';
		return code;
	},
	javascriptGenerator: (block, _js) => {
		var xPos = block.getFieldValue('x_pos') as number;
		var yPos = block.getFieldValue('y_pos') as number;
		var zPos = block.getFieldValue('z_pos') as number;
		var speed = block.getFieldValue('speed') as number;

		return `simulator.goToXyzSpeed(groupState, ${xPos}, ${yPos}, ${zPos}, ${speed})\n`;
	},
};