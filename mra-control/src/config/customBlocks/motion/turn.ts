/* eslint-disable @typescript-eslint/no-unused-vars */
/* eslint-disable @typescript-eslint/no-unsafe-call */
import {type RobartBlockDefinition} from '../BlockDefinition';
import Blockly from 'blockly';
import * as SIM from '@MRAControl/state/simulatorCommands';

export const blockTurn: RobartBlockDefinition = {
	name: 'turn',
	block:{
		init: function () {
			this.appendDummyInput()
				.appendField('turn')
				.appendField(new Blockly.FieldDropdown([['right', '\n\t\t\t\t\t\t\t\t\tright\n\t\t\t\t\t\t\t\t'], ['left', 'left']]), 'direction')
				.appendField(new Blockly.FieldAngle(0), 'degrees')
				.appendField('at')
				.appendField(new Blockly.FieldAngle(0), 'rate')
				.appendField('degrees/s');
			this.setPreviousStatement(true, null);
			this.setNextStatement(true, null);
			this.setColour(90);
			this.setTooltip('');
			this.setHelpUrl('');
		},
	},
	pythonGenerator: (block, _python) => {
		//TODO: add python equivalent
		var direction = block.getFieldValue('direction') as number;
		var degrees = block.getFieldValue('degrees') as number;
		var rate = block.getFieldValue('rate') as number;
		var code = 'turn(' + direction + ',' + degrees + ',' + rate + ')\n';
		return code;
	},
	javascriptGenerator: (_block, _js) => {
		return 'simulator.dummy();';
	},
};