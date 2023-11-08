/* eslint-disable @typescript-eslint/no-unused-vars */
/* eslint-disable @typescript-eslint/no-unsafe-call */
import {type RobartBlockDefinition} from '../BlockDefinition';
import Blockly from 'blockly';
import * as SIM from '@MRAControl/state/simulatorCommands';

export const blockStartLinearMotion: RobartBlockDefinition = {
	name: 'start_linear_motion',
	block:{
		init: function () {
			this.appendDummyInput()
				.appendField('Start linear motion with speeds in');
			this.appendDummyInput()
				.appendField('x direction:')
				.appendField(new Blockly.FieldNumber(0), 'velocity_x_m')
				.appendField('m/s');
			this.appendDummyInput()
				.appendField('y direction:')
				.appendField(new Blockly.FieldNumber(0), 'velocity_y_m')
				.appendField('m/s');
			this.appendDummyInput()
				.appendField('z direction:')
				.appendField(new Blockly.FieldNumber(0), 'velocity_z_m')
				.appendField('m/s');
			this.appendDummyInput()
				.appendField('yaw:')
				.appendField(new Blockly.FieldAngle(90), 'rate_yaw')
				.appendField('/s');
			this.setColour(90);
			this.setTooltip('');
			this.setHelpUrl('');
		},
	},
	pythonGenerator: (block, _python) => {
		var velX = block.getFieldValue('velocity_x_m') as number;
		var velY = block.getFieldValue('velocity_y_m') as number;
		var velZ = block.getFieldValue('velocity_z_m') as number;
		var rateYaw = block.getFieldValue('rate_yaw') as number;
		var code = 'start_linear_motion(' + velX + ',' + velY + ',' + velZ + ',' + rateYaw + ')\n';
		return code;
	},
	javascriptGenerator: (_block, _js) => {
		return 'simulator.dummy();\n';
	},
};