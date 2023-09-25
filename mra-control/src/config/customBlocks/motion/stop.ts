import {type RobartBlockDefinition} from '../BlockDefinition';
import Blockly from 'blockly';
import * as SIM from '@MRAControl/state/simulatorCommands';

export const block_stop: RobartBlockDefinition = {
	name: 'stop',
	block:{
		init: function () {
			this.appendDummyInput()
				.appendField('stop and hover');
			this.setPreviousStatement(true, null);
			this.setNextStatement(true, null);
			this.setColour(0);
			this.setTooltip('');
			this.setHelpUrl('');
		},
	},
	pythonGenerator: (block, python) => {
		return 'stop_and_hover(cf)\n';
	},
	javascriptGenerator: (block, js) => {
		return 'duration += simulator.stop_and_hover(groupState);\n';
	},
	execute: (block, groupState) => {
		const simulator = SIM;
		return 0.1;
		// TODO return simulator.stop_and_hover(groupState)
	},
};