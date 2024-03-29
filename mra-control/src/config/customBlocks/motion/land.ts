/* eslint-disable @typescript-eslint/no-unused-vars */
/* eslint-disable @typescript-eslint/no-unsafe-assignment */
/* eslint-disable @typescript-eslint/no-unsafe-call */
import {type RobartBlockDefinition} from '../BlockDefinition';
import Blockly from 'blockly';
import * as SIM from '@MRAControl/state/simulatorCommands';

export const blockLand: RobartBlockDefinition = {
	name: 'land',
	block:{
		init: function () {
			this.appendDummyInput()
				.appendField('land at height')
				.appendField(new Blockly.FieldNumber(0, 0), 'height')
				.appendField('over')
				.appendField(new Blockly.FieldNumber(3, 0.1), 'duration')
				.appendField('seconds');
			this.setPreviousStatement(true, null);
			this.setNextStatement(true, null);
			this.setColour(230);
			this.setTooltip('Land at a certain height.');
			this.setHelpUrl('');
		},
	},
	pythonGenerator: (block, _python) => {
		var numberHeight = block.getFieldValue('height') as number;
		var duration = block.getFieldValue('duration') as number;
		var code = 'land(groupState, ' + numberHeight + ',' + duration + ')\n';
		return code;
	},
	javascriptGenerator: (block, _js) => {
		var height = block.getFieldValue('height');
		var duration = block.getFieldValue('duration');
		return `simulator.land(groupState, ${height}, ${duration})\n`;
	},
};