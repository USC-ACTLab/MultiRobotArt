/* eslint-disable @typescript-eslint/no-unused-vars */
/* eslint-disable @typescript-eslint/no-unsafe-call */
import {type RobartBlockDefinition} from '../BlockDefinition';
import Blockly from 'blockly';

export const blockCircleArc: RobartBlockDefinition = {
	name: 'circleArc',
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
				.appendField('Starting at')
				.appendField(new Blockly.FieldNumber(90), 'angle_degrees_start')
				.appendField('degrees,');
			this.appendDummyInput()
				.appendField('Ending at')
				.appendField(new Blockly.FieldNumber(90), 'angle_degrees_end')
				.appendField('degrees');
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
		const radius = block.getFieldValue('radius_m') as number;
		const velocity = block.getFieldValue('velocity') as number;
		const degrees = block.getFieldValue('angle_degrees') as number;
		const dropDownDirection = block.getFieldValue('direction') as string;
		const code = 'circle(groupState' + radius + ',' + velocity + ',' + degrees + ',\'' + dropDownDirection + '\')';
		return code; //TODO, method doesn't exit!
	},
	javascriptGenerator: (block, _js) => {
		const radius = block.getFieldValue('radius_m') as number;
		const velocity = block.getFieldValue('velocity') as number;
		const degreesStart = block.getFieldValue('angle_degrees_start') as number;
		const degreesEnd = block.getFieldValue('angle_degrees_end') as number;
		const dropDownDirection = block.getFieldValue('direction') as string;	
		let clockwise = true;
		if (dropDownDirection == 'ccw') {
			clockwise = false;
		}

		return `simulator.moveCircleArcVel(groupState, ${radius}, ${velocity}, ${degreesStart}, ${degreesEnd}, ${clockwise})\n`;
	},
};