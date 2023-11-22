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

export const blockColor: RobartBlockDefinition = {
	name: 'color',
	block:{
		init: function () {
			this.appendDummyInput()
				.appendField('set LED color:')
				.appendField(new ColorWheelField('#00FF00', 150, {
					layoutDirection: 'horizontal',
				}), 'color');
			this.setPreviousStatement(true, null);
			this.setNextStatement(true, null);
		},
    
	},

	pythonGenerator: (block, _python) => {
		const color = block.getFieldValue('color') as string;
		const code = 'setLEDColorFromHex(cf, "' + color + '")\n';
		return code;
	},



	javascriptGenerator: (block, _js) => {
		var color = block.getFieldValue('color') as string;
		var rgbCol = hexToRgb(color);
		return `simulator.setColor(groupState, ${rgbCol?.r},${rgbCol?.g},${rgbCol?.b})\n`;

	},
};
export const blockRandomColor: RobartBlockDefinition = {
	name: 'randomColor',
	block:{
		init: function () {
			this.appendDummyInput()
				.appendField('Random LED Color:');
			this.setPreviousStatement(true, null);
			this.setNextStatement(true, null);
		},
    
	},

	pythonGenerator: (block, _python) => {
		const color = block.getFieldValue('color') as string;
		const code = 'setLEDColorFromHex(cf, "' + color + '")\n';
		return code;
	},



	javascriptGenerator: (block, _js) => {
		var color = {r: Math.random(), b: Math.random(), g: Math.random()};
		return `simulator.setColor(groupState, ${color?.r},${color?.g},${color?.b})\n`;
	},
};
