/* eslint-disable @typescript-eslint/no-unused-vars */
/* eslint-disable @typescript-eslint/no-unsafe-call */
import {type RobartBlockDefinition} from '../BlockDefinition';
import Blockly from 'blockly';
import * as SIM from '@MRAControl/state/simulatorCommands';

export const blockMove: RobartBlockDefinition = {
	name: 'move',
	block: {
		init: function () {
			this.appendDummyInput()
				.appendField('move')
				.appendField(new Blockly.FieldDropdown([['up', 'up'], ['down', 'down'], ['left', 'left'], ['right', 'right'], ['forward', 'forward'], ['backward', 'backward']]), 'direction')
				.appendField(new Blockly.FieldNumber(0), 'distance')
				.appendField('meters for')
				.appendField(new Blockly.FieldNumber(0), 'duration')
				.appendField('seconds');
			this.setPreviousStatement(true, null);
			this.setNextStatement(true, null);
			this.setColour(90);
			this.setTooltip('');
			this.setHelpUrl('');
		},
	},
	pythonGenerator: (block, _python) => {
		var direction = block.getFieldValue('direction') as number;
		var distance = block.getFieldValue('distance') as number;
		var duration = block.getFieldValue('duration') as number;
		var code = 'move_direction(cf, ' + direction + ',' + distance + ',' + duration + ')\n';
		return code;
	},
	javascriptGenerator: (block, _js) => {
		var direction = block.getFieldValue('direction') as number;
		var distance = block.getFieldValue('distance') as number;
		var duration = block.getFieldValue('duration') as number;
		return `simulator.move_direction(groupState, ${direction}, ${distance}, ${duration})\n`;
	},
};