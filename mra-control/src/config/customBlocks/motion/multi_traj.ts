/* eslint-disable @typescript-eslint/no-unsafe-call */
import {type RobartBlockDefinition} from '../BlockDefinition';
import Blockly from 'blockly';
import * as SIM from '@MRAControl/state/simulatorCommands';
import {string} from 'blockly/core/utils';

export const multiTraj: RobartBlockDefinition = {
	name: 'multiTraj',
	block:{
		init: function () {
			this.appendDummyInput()
				.appendField('Trajectory by components');
            
			this.appendStatementInput('x')
				.appendField('X:');
			// this.appendField("y");
			this.appendStatementInput('y')
				.appendField('Y:');
			// this.appendField("z");
			this.appendStatementInput('z')
				.appendField('Z:');
			this.setPreviousStatement(true, null);
			this.setNextStatement(true, null);
			this.setColour(90);
			this.setTooltip('');
			this.setHelpUrl('');
		},
	},
	pythonGenerator: (block, python) => {
		// var number_velocity_x_m = block.getFieldValue('velocity_x_m');
		// var number_velocity_y_m = block.getFieldValue('velocity_y_m');
		// var number_velocity_z_m = block.getFieldValue('velocity_z_m');
		// var angle_rate_yaw = block.getFieldValue('rate_yaw');
		// var code = 'start_linear_motion(' + number_velocity_x_m + ',' + number_velocity_y_m + ',' + number_velocity_z_m + ',' + angle_rate_yaw + ')\n';
		const code = 'pass';
		return code;
	},
	javascriptGenerator: (block, js) => {
		var x_traj = js.statementToCode(block, 'x');
		var y_traj = js.statementToCode(block, 'y');
		var z_traj = js.statementToCode(block, 'z');

        console.log(x_traj);
        // If no trajectory is given, use a dummy
		if (x_traj.length === 0) {
			x_traj = 'simulator.dummy()';
		}if (y_traj.length === 0) {
			y_traj = 'simulator.dummy()';
		}if (z_traj.length === 0) {
			z_traj = 'simulator.dummy()';
		}


		var code = `simulator.componentTraj(groupState, ${x_traj}, ${y_traj}, ${z_traj})\n`;
		console.log(code);

		return code;
	},
};