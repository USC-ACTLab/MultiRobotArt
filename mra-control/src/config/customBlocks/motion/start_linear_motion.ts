import { RobartBlockDefinition } from '../BlockDefinition';
import Blockly from 'blockly';

export const block_start_linear_motion: RobartBlockDefinition = {
  name: "start_linear_motion",
  block:{
    init: function() {
      this.appendDummyInput()
          .appendField("Start linear motion with speeds in");
      this.appendDummyInput()
          .appendField("x direction:")
          .appendField(new Blockly.FieldNumber(0), "velocity_x_m")
          .appendField("m/s");
      this.appendDummyInput()
          .appendField("y direction:")
          .appendField(new Blockly.FieldNumber(0), "velocity_y_m")
          .appendField("m/s");
      this.appendDummyInput()
          .appendField("z direction:")
          .appendField(new Blockly.FieldNumber(0), "velocity_z_m")
          .appendField("m/s");
      this.appendDummyInput()
          .appendField("yaw:")
          .appendField(new Blockly.FieldAngle(90), "rate_yaw")
          .appendField("/s");
      this.setColour(90);
   this.setTooltip("");
   this.setHelpUrl("");
    }
  },
  pythonGenerator: (block, python) => {
    var number_velocity_x_m = block.getFieldValue('velocity_x_m');
    var number_velocity_y_m = block.getFieldValue('velocity_y_m');
    var number_velocity_z_m = block.getFieldValue('velocity_z_m');
    var angle_rate_yaw = block.getFieldValue('rate_yaw');
    var code = 'start_linear_motion(' + number_velocity_x_m + ',' + number_velocity_y_m + ',' + number_velocity_z_m + ',' + angle_rate_yaw + ')\n';
    return code;
  },
  javascriptGenerator: (block, js) => {
    return `simulator.dummy();`;
  }
}