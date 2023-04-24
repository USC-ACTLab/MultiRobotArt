import { RobartBlockDefinition } from '../BlockDefinition';
import Blockly from 'blockly';

export const block_start_turn: RobartBlockDefinition = {
  name: "start_turn",
  block:{
    init: function() {
      this.appendDummyInput()
        .appendField("start turning")
        .appendField(new Blockly.FieldDropdown([["right","right"], ["left","left"]]), "direction")
        .appendField("at")
        .appendField(new Blockly.FieldAngle(0), "rate")
        .appendField("degrees/s");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(90);
 this.setTooltip("");
 this.setHelpUrl("");
    }
  },
  pythonGenerator: (block, python) => {
    var dropdown_direction = block.getFieldValue('direction');
  var angle_rate = block.getFieldValue('rate');
  var code = 'start_turn(' + dropdown_direction + ',' + angle_rate + ')\n';
  return code;
  },
  javascriptGenerator: (block, js) => {
    return `simulator.dummy();`;
  }
}