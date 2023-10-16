/* eslint-disable @typescript-eslint/no-unused-vars */
/* eslint-disable @typescript-eslint/no-unsafe-call */
import {type RobartBlockDefinition} from '../BlockDefinition';
import Blockly from 'blockly';

export const blockCircle: RobartBlockDefinition = {
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
	pythonGenerator: (block, _python) => {
		var radius = block.getFieldValue('radius_m') as number;
		var velocity = block.getFieldValue('velocity') as number;
		var angle = block.getFieldValue('angle_degrees') as number;
		var dropdownDirection = block.getFieldValue('direction') as number;
		var code = 'circle(' + radius + ',' + velocity + ',' + angle + ',\'' + dropdownDirection + '\')';
		return code;
	},
	javascriptGenerator: (block, _js) => {
		const radius = block.getFieldValue('radius_m') as number;
		const velocity = block.getFieldValue('velocity') as number;
		const angle = block.getFieldValue('angle_degrees') as number;
		const direction = block.getFieldValue('direction') as string;
		var clockwise = true;
		if (direction == 'ccw') {
			clockwise = false;
		}

		return `simulator.moveCircleVel(groupState, ${radius}, ${velocity}, ${angle}, ${clockwise})\n`;
	},
};