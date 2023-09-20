import { RobartBlockDefinition } from '../BlockDefinition';
import Blockly from 'blockly';

export const block_move_xyz: RobartBlockDefinition = {
  name: "move_xyz",
  block: {
    init: function () {
      this.appendDummyInput()
      .appendField("move X")
      .appendField(new Blockly.FieldNumber(0), "x")
      .appendField("meters");
  this.appendDummyInput()
      .appendField("move Y")
      .appendField(new Blockly.FieldNumber(0), "y")
      .appendField("meters");
  this.appendDummyInput()
      .appendField("move Z")
      .appendField(new Blockly.FieldNumber(0), "z")
      .appendField("meters");
  this.appendDummyInput()
      .appendField("at")
      .appendField(new Blockly.FieldNumber(0), "speed")
      .appendField("m/s");
  this.setPreviousStatement(true, null);
  this.setNextStatement(true, null);
  this.setColour(90);
this.setTooltip("");
this.setHelpUrl("");
    },
  },
  pythonGenerator: (block, python) => {
    var number_x = block.getFieldValue('x');
    var number_y = block.getFieldValue('y');
    var number_z = block.getFieldValue('z');
    var number_speed = block.getFieldValue('speed');
    var code = 'goto_rel_at_speed(cf, ' + number_x + ',' + number_y + ',' + number_z + ',' + number_speed + ')\n';
    return code;
  },
  javascriptGenerator: (block, js) => {
    var number_x = block.getFieldValue('x');
    var number_y = block.getFieldValue('y');
    var number_z = block.getFieldValue('z');
    var number_speed = block.getFieldValue('speed');
    return `duration += simulator.move_speed(group_state, ${number_x}, ${number_y}, ${number_z}, ${number_speed});\n`;
  }
}