/* eslint-disable @typescript-eslint/no-unused-vars */
/* eslint-disable @typescript-eslint/no-unsafe-call */
import {type RobartBlockDefinition} from '../BlockDefinition';
import Blockly from 'blockly';
import * as SIM from '@MRAControl/state/simulatorCommands';

export const blockStop: RobartBlockDefinition = {
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
	pythonGenerator: (_block, _python) => {
		return 'stop_and_hover(groupState)\n';
	},
	javascriptGenerator: (_block, _js) => {
		return 'simulator.stop_and_hover(groupState)\n';
	},
};