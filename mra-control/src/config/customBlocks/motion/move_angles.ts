import { RobartBlockDefinition } from '../BlockDefinition';
import Blockly from 'blockly';

export const block_move_angles: RobartBlockDefinition = {
  name: "move_angles",
  block: {
    init: function () {
      this.appendDummyInput()
      .appendField("move towards");
  this.appendDummyInput()
      .appendField("horizontal angle")
      .appendField(new Blockly.FieldAngle(90), "horizontal angle");
  this.appendDummyInput()
      .appendField("vertical angle")
      .appendField(new Blockly.FieldAngle(90), "vertical angle");
  this.appendDummyInput()
      .appendField("distance")
      .appendField(new Blockly.FieldNumber(0), "distance");
  this.appendDummyInput()
      .appendField("at")
      .appendField(new Blockly.FieldNumber(0), "NAME")
      .appendField("m/s");
  this.setColour(90);
this.setTooltip("");
this.setHelpUrl("");
    },
  },
  pythonGenerator: (block, python) => {
    var angle_horizontal_angle = block.getFieldValue('horizontal angle');
    var angle_vertical_angle = block.getFieldValue('vertical angle');
    var number_distance = block.getFieldValue('distance');
    var number_name = block.getFieldValue('NAME');
    var code = 'move_by_angles(' + angle_horizontal_angle + ',' + angle_vertical_angle + ',' + number_distance + ',' + number_name + ')\n';
    return code;
  },
  javascriptGenerator: (block, js) => {
    return `duration += simulator.dummy()`;
  }
}