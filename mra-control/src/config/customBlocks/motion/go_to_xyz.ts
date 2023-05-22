import { RobartBlockDefinition } from '../BlockDefinition';
import Blockly from 'blockly';

export const block_go_to_xyz: RobartBlockDefinition = {
  name: "go_to_xyz",
  block: {
    init: function () {
      this.appendDummyInput()
      .appendField("go to");
  this.appendValueInput("x")
      .setCheck("Number")
      .appendField("----------------X:");
  this.appendValueInput("y")
      .setCheck("Number")
      .appendField("----------------Y:");
  this.appendValueInput("z")
      .setCheck("Number")
      .appendField("----------------Z:");
  this.appendValueInput("speed")
      .setCheck("Number")
      .appendField("at speed (m/s)");
  this.setInputsInline(false);
  this.setPreviousStatement(true, null);
  this.setNextStatement(true, null);
  this.setColour(90);
this.setTooltip("");
this.setHelpUrl("");
    },
  },
  pythonGenerator: (block, python) => {
    var value_x = Blockly.JavaScript.valueToCode(block, 'x', Blockly.JavaScript.ORDER_ATOMIC);
  var value_y = Blockly.JavaScript.valueToCode(block, 'y', Blockly.JavaScript.ORDER_ATOMIC);
  var value_z = Blockly.JavaScript.valueToCode(block, 'z', Blockly.JavaScript.ORDER_ATOMIC);
  var value_speed = Blockly.JavaScript.valueToCode(block, 'speed', Blockly.JavaScript.ORDER_ATOMIC);
  var code = 'go_to_xyz(' + value_x + ',' + value_y + ',' + value_z + ',' + value_speed + ')\n';
  return code;
  },
  javascriptGenerator: (block, js) => {
    var x = js.valueToCode(block, 'x', js.ORDER_ATOMIC);
    var y = js.valueToCode(block, 'y', js.ORDER_ATOMIC);
    var z = js.valueToCode(block, 'z', js.ORDER_ATOMIC);
    var speed = js.valueToCode(block, 'speed', js.ORDER_ATOMIC);

    return `simulator.go_to_xyz(group_state, ${x}, ${y}, ${z}, ${speed})`;
  }
}