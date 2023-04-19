import { RobartBlockDefinition } from '../BlockDefinition';
import Blockly from 'blockly';

export const block_set_default_speed: RobartBlockDefinition = {
  name: "set_default_speed",
  block:{
    init: function() {
        this.appendDummyInput()
        .appendField("set default speed to")
        .appendField(new Blockly.FieldNumber(0), "speed");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
    this.setTooltip("");
    this.setHelpUrl("");
    }
  },
  pythonGenerator: (block, python) => {
    var number_speed = block.getFieldValue('speed');
    var code = 'set_default_speed(' + number_speed + ')\n';
    return code;
  },
  javascriptGenerator: (block, js) => {
    return `simulator.dummy();`;
  }
}