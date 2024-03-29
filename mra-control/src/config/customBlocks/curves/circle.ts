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
				.appendField(new Blockly.FieldNumber(1, 0.01), 'radius_m')
				.appendField('m');
			this.appendDummyInput()
				.appendField('at speed')
				.appendField(new Blockly.FieldNumber(0.5, 0.01), 'velocity')
				.appendField('m/s');
			this.appendDummyInput()
				.appendField('for')
				.appendField(new Blockly.FieldNumber(360), 'angle_degrees')
				.appendField('degrees,');
			this.setPreviousStatement(true, null);
			this.setNextStatement(true, null);
			this.setColour(90);
			this.setTooltip('Draw a circle in the YZ-Plane with given parameters. The circle begins at angle=0 and proceeds counter-clockwise until the desired angle is reached. Note that an angle of 0 is on the right side of the circle (Think Unit Circle).');
			this.setHelpUrl('https://en.wikipedia.org/wiki/Unit_circle');
		},
	},
	pythonGenerator: (block, _python) => {
		const radius = block.getFieldValue('radius_m') as number;
		const velocity = block.getFieldValue('velocity') as number;
		const degrees = block.getFieldValue('angle_degrees') as number;
		const dropDownDirection = block.getFieldValue('direction') as string;

		const radians = degrees / 180 * Math.PI;
		const code = 'circle(groupState, ' + radius + ', ' + velocity + ', ' + radians + ',\'' + dropDownDirection + '\')';
		return code;
	},
	javascriptGenerator: (block, _js) => {
		const radius = block.getFieldValue('radius_m') as number;
		const velocity = block.getFieldValue('velocity') as number;
		const degrees = block.getFieldValue('angle_degrees') as number;
		// const dropDownDirection = block.getFieldValue('direction') as string;	
		let clockwise = true;
		// if (dropDownDirection == 'ccw') {
		// 	clockwise = false;
		// }

		return `simulator.moveCircleVel(groupState, ${radius}, ${velocity}, ${degrees}, ${clockwise})\n`;
	},
};

export const blockCircleRadians: RobartBlockDefinition = {
	name: 'circle_radians',
	block:{
		init: function () {
			this.appendDummyInput()
				.appendField(new Blockly.FieldLabelSerializable('Go in circle of radius'), 'Circular motion')
				.appendField(new Blockly.FieldNumber(1, 0), 'radius_m')
				.appendField('m');
			this.appendDummyInput()
				.appendField('at speed')
				.appendField(new Blockly.FieldNumber(0.5), 'velocity')
				.appendField('m/s');
			this.appendDummyInput()
				.appendField('for')
				.appendField(new Blockly.FieldNumber(6.28), 'angle_radians')
				.appendField('radians,');
			// this.appendDummyInput()
			// 	.appendField(new Blockly.FieldDropdown([['counterclockwise', 'ccw'], ['clockwise', 'cw']]), 'direction');
			this.setPreviousStatement(true, null);
			this.setNextStatement(true, null);
			this.setColour(90);
			this.setTooltip('Draw a circle in the YZ-Plane with given parameters. The circle begins at angle=0 and proceeds counter-clockwise until the desired angle is reached. Note that an angle of 0 is on the right side of the circle (Think Unit Circle).');
			this.setHelpUrl('');
		},
	},
	pythonGenerator: (block, _python) => {
		const radius = block.getFieldValue('radius_m') as number;
		const velocity = block.getFieldValue('velocity') as number;
		const radians = block.getFieldValue('angle_radians') as number;
		// const dropDownDirection = block.getFieldValue('direction') as string;

		const code = 'circle(groupState, ' + radius + ', ' + velocity + ', ' + radians + ',\'' + 'clockwise' + '\')';
		return code;
	},
	javascriptGenerator: (block, _js) => {
		const radius = block.getFieldValue('radius_m') as number;
		const velocity = block.getFieldValue('velocity') as number;
		const radians = block.getFieldValue('angle_radians') as number;
		let clockwise = true;
		// if (dropDownDirection == 'ccw') {
		// 	clockwise = false;
		// }

		return `simulator.moveCircleVel(groupState, ${radius}, ${velocity}, ${radians*180/Math.PI}, ${clockwise})\n`;
	},
};