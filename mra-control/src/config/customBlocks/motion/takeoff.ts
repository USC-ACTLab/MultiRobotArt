/* eslint-disable @typescript-eslint/no-unused-vars */
/* eslint-disable @typescript-eslint/no-unsafe-assignment */
/* eslint-disable @typescript-eslint/no-unsafe-call */
import {type RobartBlockDefinition} from '../BlockDefinition';
import Blockly from 'blockly';
import * as SIM from '@MRAControl/state/simulatorCommands';

export const blockTakeoff: RobartBlockDefinition = {
	name: 'takeoff',
	block:{
		init: function () {
			this.appendDummyInput()
				.appendField('takeoff to')
				.appendField(new Blockly.FieldNumber(0), 'height')
				.appendField('meters over')
				.appendField(new Blockly.FieldNumber(0), 'duration')
				.appendField('seconds');
			this.setPreviousStatement(false, null);
			this.setNextStatement(true, null);
			this.setColour(230);
			this.setTooltip('');
			this.setHelpUrl('');
		},
	},
	pythonGenerator: (block, _python) => {
		var height = block.getFieldValue('height') as number;
		var duration = block.getFieldValue('duration') as number;
		var code = 'takeoff(cf, ' + height + ' ,' + duration + ')\n';
		return code;
	},
	javascriptGenerator: (block, _js) => {
		var height = block.getFieldValue('height');
		var duration = block.getFieldValue('duration');
		return `simulator.takeoff(groupState, ${height}, ${duration})\n`;
	},
};