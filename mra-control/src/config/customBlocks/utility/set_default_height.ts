import { RobartBlockDefinition } from '../BlockDefinition';
import Blockly from 'blockly';

export const block_set_default_height: RobartBlockDefinition = {
  name: "set_default_height",
  block:{
    init: function() {
      this.appendDummyInput()
        .appendField("set default height to")
        .appendField(new Blockly.FieldNumber(0, 0), "default_height")
        .appendField("meters");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
  this.setTooltip("");
  this.setHelpUrl("");
    }
  },
  pythonGenerator: (block, python) => {
    var number_default_height = block.getFieldValue('default_height');
    var code = 'set_default_height(' + number_default_height + ')\n';
    return code;
  },
  javascriptGenerator: (block, js) => {
    return `duration += simulator.dummy();`;
  }
}