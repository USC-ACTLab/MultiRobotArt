/* eslint-disable @typescript-eslint/no-unused-vars */
/* eslint-disable @typescript-eslint/no-unsafe-call */
import {type RobartBlockDefinition} from '../BlockDefinition';
import Blockly from 'blockly';
import * as SIM from '@MRAControl/state/simulatorCommands';
import {Simulation} from '@MRAControl/layout/simulation/Simulation';

export const blockMoveTowards: RobartBlockDefinition = {
	name: 'move_towards',
	block: {
		init: function () {
			this.appendDummyInput()
				.appendField('Move ')
				.appendField(new Blockly.FieldNumber(1, 0.01))
				.appendField('meter in the direction of X:')
				.appendField(new Blockly.FieldNumber(0), 'x_pos')
				.appendField('Y:')
				.appendField(new Blockly.FieldNumber(0), 'y_pos')
				.appendField('Z:')
				.appendField(new Blockly.FieldNumber(0), 'z_pos')
				.appendField('over')
				.appendField(new Blockly.FieldNumber(3, 0.1), 'duration')
				.appendField('seconds');
			this.setPreviousStatement(true, null);
			this.setNextStatement(true, null);
			this.setColour(90);
			this.setTooltip('Move in a desired direction for a fixed duration. (x, y, z) correspond to global coordinates, so unlike the standard "move" block.');
			this.setHelpUrl('');
		},
	},
	pythonGenerator: (block, _python) => {
		var xPosition = block.getFieldValue('x_pos') as number;
		var yPosition = block.getFieldValue('y_pos') as number;
		var zPosition = block.getFieldValue('z_pos') as number;
		var duration = block.getFieldValue('duration') as number;
		var code = 'goto_duration(groupState, ' + xPosition + ',' + yPosition + ',' + zPosition + ',' + duration + ')\n';
		return code;
	},
	javascriptGenerator: (block, _js) => {
		var xPosition = block.getFieldValue('x_pos') as number;
		var yPosition = block.getFieldValue('y_pos') as number;
		var zPosition = block.getFieldValue('z_pos') as number;
		var duration = block.getFieldValue('duration') as number;

		return `simulator.goToXyzDuration(groupState, ${xPosition}, ${yPosition}, ${zPosition}, ${duration})\n`;
	},
};