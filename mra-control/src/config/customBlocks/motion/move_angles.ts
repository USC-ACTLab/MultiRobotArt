import {type RobartBlockDefinition} from '../BlockDefinition';
import Blockly from 'blockly';

import * as SIM from '@MRAControl/state/simulatorCommands';

export const block_move_angles: RobartBlockDefinition = {
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
	pythonGenerator: (block, python) => {
		var angle_horizontal_angle = block.getFieldValue('horizontal angle');
		var angle_vertical_angle = block.getFieldValue('vertical angle');
		var number_distance = block.getFieldValue('distance');
		var number_name = block.getFieldValue('speed');
		var code = 'move_by_angles(' + angle_horizontal_angle + ',' + angle_vertical_angle + ',' + number_distance + ',' + number_name + ')\n';
		return code;
	},
	javascriptGenerator: (block, js) => {
		var angle_horizontal_angle = block.getFieldValue('horizontal angle');
		var angle_vertical_angle = block.getFieldValue('vertical angle');
		var number_distance = block.getFieldValue('distance');
		var number_speed = block.getFieldValue('speed');
		return `duration += simulator.move_by_angles(groupState, ${angle_horizontal_angle}, ${angle_vertical_angle}, ${number_distance}, ${number_speed})`;
	},
	execute: (block, groupState) => {
		//TODO...
		const simulator = SIM;
		var angle_horizontal_angle = block.getFieldValue('horizontal angle');
		var angle_vertical_angle = block.getFieldValue('vertical angle');
		var number_distance = block.getFieldValue('distance');
		var number_speed = block.getFieldValue('speed');
		return 0.1;
		//return simulator.move_by_angles(groupState, angle_horizontal_angle, angle_vertical_angle, number_distance, number_speed)
	},
};