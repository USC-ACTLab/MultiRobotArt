import {type RobartBlockDefinition} from '../BlockDefinition';
import Blockly from 'blockly';
import * as SIM from '@MRAControl/state/simulatorCommands';

export const block_circle: RobartBlockDefinition = {
	name: 'circle',
	block:{
		init: function () {
			this.appendDummyInput()
				.appendField(new Blockly.FieldLabelSerializable('Go in circle of radius'), 'Circular motion')
				.appendField(new Blockly.FieldNumber(0, 0), 'radius_m')
				.appendField('m');
			this.appendDummyInput()
				.appendField('at speed')
				.appendField(new Blockly.FieldNumber(0.2), 'velocity')
				.appendField('m/s');
			this.appendDummyInput()
				.appendField('for')
				.appendField(new Blockly.FieldAngle(90), 'angle_degrees')
				.appendField('degrees,');
			this.appendDummyInput()
				.appendField(new Blockly.FieldDropdown([['counterclockwise', 'ccw'], ['clockwise', 'cw']]), 'direction');
			this.setPreviousStatement(true, null);
			this.setNextStatement(true, null);
			this.setColour(90);
			this.setTooltip('');
			this.setHelpUrl('');
		},
	},
	pythonGenerator: (block, python) => {
		var number_radius_m = block.getFieldValue('radius_m');
		var number_velocity = block.getFieldValue('velocity');
		var angle_angle_degrees = block.getFieldValue('angle_degrees');
		var dropdown_direction = block.getFieldValue('direction');
		var code = 'circle(' + number_radius_m + ',' + number_velocity + ',' + angle_angle_degrees + ',\'' + dropdown_direction + '\')';
		return code;
	},
	javascriptGenerator: (block, js) => {
		const number_radius_m = block.getFieldValue('radius_m');
		const number_velocity = block.getFieldValue('velocity');
		const angle_degrees = block.getFieldValue('angle_degrees');
		const direction = block.getFieldValue('direction');
		var clockwise = true;
		if (direction == 'ccw') {
			clockwise = false;
		}

		return `simulator.moveCircleVel(groupState, ${number_radius_m}, ${number_velocity}, ${angle_degrees}, ${clockwise});\n`;
	},
	execute: (block, groupState) => {
		const simulator = SIM;
		const number_radius_m = block.getFieldValue('radius_m');
		const number_velocity = block.getFieldValue('velocity');
		const angle_degrees = block.getFieldValue('angle_degrees');
		const direction = block.getFieldValue('direction');
		var clockwise = true;
		if (direction == 'ccw') {
			clockwise = false;
		}

		return simulator.moveCircleVel(groupState, number_radius_m, number_velocity, angle_degrees, clockwise);
	},
};