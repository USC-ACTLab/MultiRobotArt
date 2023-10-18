/* eslint-disable @typescript-eslint/no-unused-vars */
/* eslint-disable @typescript-eslint/no-unsafe-call */
import {type RobartBlockDefinition} from '../BlockDefinition';
import Blockly from 'blockly';

export const blockGetPosition: RobartBlockDefinition = {
	name: 'get_a_position',
	block: {
		init: function () {
			this.appendDummyInput()
				.appendField(new Blockly.FieldDropdown([['default', 'default'], ['current', 'current']]), 'which')
				.appendField(new Blockly.FieldDropdown([['x', 'x position'], ['y', 'y position'], ['z', 'z position']]), 'NAME')
				.appendField('position');
			this.setOutput(true, null);
			this.setColour(230);
			this.setTooltip('');
			this.setHelpUrl('');
		},
	},
	pythonGenerator: (block, _python) => {
		//TODO Add python equivalent
		var which = block.getFieldValue('which') as string;
		var name = block.getFieldValue('NAME') as string;
		var code = 'get_a_position(' + which + ',' + name + ')';
		// TODO: Change ORDER_NONE to the correct strength.
		return code;
	},
	javascriptGenerator: (_block, _js) => {
		return 'simulator.dummy()';
	},
};