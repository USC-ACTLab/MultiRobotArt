/* eslint-disable @typescript-eslint/no-unused-vars */
/* eslint-disable @typescript-eslint/no-unsafe-assignment */
/* eslint-disable @typescript-eslint/no-unsafe-call */
import {type RobartBlockDefinition} from '../BlockDefinition';
import {ColorWheelField} from './ColorWheel';
import * as SIM from '@MRAControl/state/simulatorCommands';

// Blockly.Blocks["color_wheel_picker"] = {
//   init: function () {
//     this.appendDummyInput()
//       .appendField("Color: ")
//       .appendField(new ColorPickerField("#00FF00", 150, {
//         layoutDirection: 'horizontal',
//       }), "COLOR")
//   }
// };
function hexToRgb(hex: string) {
	var result = /^#?([a-f\d]{2})([a-f\d]{2})([a-f\d]{2})$/i.exec(hex);
	return result ? {
		r: parseInt(result[1], 16),
		g: parseInt(result[2], 16),
		b: parseInt(result[3], 16),
	} : null;
}

export const blockColorOff: RobartBlockDefinition = {
	name: 'colorOff',
	block:{
		init: function () {
			this.appendDummyInput()
				.appendField('Turn LED Off');
			this.setPreviousStatement(true, null);
			this.setNextStatement(true, null);
		},
    
	},

	pythonGenerator: (block, _python) => {
		const code = 'setLEDColor(groupState, 0, 0, 0)\n';
		return code;
	},



	javascriptGenerator: (block, _js) => {
		return 'simulator.setColor(groupState, 0, 0, 0)\n';
	},
};

