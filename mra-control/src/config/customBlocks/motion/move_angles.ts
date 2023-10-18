/* eslint-disable @typescript-eslint/naming-convention */
/* eslint-disable @typescript-eslint/no-unused-vars */
/* eslint-disable @typescript-eslint/no-unsafe-call */
import {type RobartBlockDefinition} from '../BlockDefinition';
import Blockly from 'blockly';

import * as SIM from '@MRAControl/state/simulatorCommands';

export const blockMoveAngles: RobartBlockDefinition = {
	name: 'move_angles',
	block: {
		init: function () {
			this.appendDummyInput()
				.appendField('move towards');
			this.appendDummyInput()
				.appendField('horizontal angle')
				.appendField(new Blockly.FieldAngle(90), 'horizontal angle');
			this.appendDummyInput()
				.appendField('vertical angle')
				.appendField(new Blockly.FieldAngle(90), 'vertical angle');
			this.appendDummyInput()
				.appendField('distance')
				.appendField(new Blockly.FieldNumber(0), 'distance')
				.appendField('meters');
			this.appendDummyInput()
				.appendField('at')
				.appendField(new Blockly.FieldNumber(0), 'speed')
				.appendField('m/s');
			this.setColour(90);
			this.setTooltip('');
			this.setHelpUrl('');
		},
	},
	pythonGenerator: (block, _python) => {
		var horizontalAngle = block.getFieldValue('horizontal angle') as number;
		var verticalAngle = block.getFieldValue('vertical angle') as number;
		var distance = block.getFieldValue('distance') as number;
		var name = block.getFieldValue('speed') as number;
		var code = 'move_by_angles(' + horizontalAngle + ',' + verticalAngle + ',' + distance + ',' + name + ')\n';
		return code;
	},
	javascriptGenerator: (block, _js) => {
		var angle_horizontal_angle = block.getFieldValue('horizontal angle') as number;
		var angle_vertical_angle = block.getFieldValue('vertical angle') as number;
		var number_distance = block.getFieldValue('distance') as number;
		var number_speed = block.getFieldValue('speed') as number;
		return `simulator.move_by_angles(groupState, ${angle_horizontal_angle}, ${angle_vertical_angle}, ${number_distance}, ${number_speed})\n`;
	},
};