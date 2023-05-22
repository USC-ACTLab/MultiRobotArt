import { RobartBlockDefinition } from '../BlockDefinition';
import Blockly from 'blockly';

export const block_go_to: RobartBlockDefinition = {
  name: "go_to",
  block: {
    init: function () {
      this.appendDummyInput()
        .appendField("go to X:")
        .appendField(new Blockly.FieldNumber(0), "x_pos")
        .appendField("Y:")
        .appendField(new Blockly.FieldNumber(0), "y_pos")
        .appendField("Z:")
        .appendField(new Blockly.FieldNumber(0), "z_pos")
        .appendField("over")
        .appendField(new Blockly.FieldNumber(0), "duration")
        .appendField("seconds");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(90);
 this.setTooltip("");
 this.setHelpUrl("");
    },
  },
  pythonGenerator: (block, python) => {
    var number_x_pos = block.getFieldValue('x_pos');
    var number_y_pos = block.getFieldValue('y_pos');
    var number_z_pos = block.getFieldValue('z_pos');
    var number_speed = block.getFieldValue('speed');
    var code = 'go_to(' + number_x_pos + ',' + number_y_pos + ',' + number_z_pos + ',' + number_speed + ')\n';
    return code;
  },
  javascriptGenerator: (block, js) => {
    var x = block.getFieldValue('x_pos');
    var y = block.getFieldValue('y_pos');
    var z = block.getFieldValue('z_pos');
    var speed = block.getFieldValue('speed');

    return `simulator.go_to_xyz(group_state, ${x}, ${y}, ${z}, ${speed})`;
  }
}