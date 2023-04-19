import { RobartBlockDefinition } from '../BlockDefinition';
import Blockly from 'blockly';

export const block_set_default_xy: RobartBlockDefinition = {
  name: "set_default_xy",
  block:{
    init: function() {
      this.appendDummyInput()
      .appendField("set default")
      .appendField(new Blockly.FieldDropdown([["x","x"], ["y","y"]]), "xy")
      .appendField("to")
      .appendField(new Blockly.FieldNumber(0), "NAME");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
    this.setTooltip("");
    this.setHelpUrl("");
    }
  },
  pythonGenerator: (block, python) => {
    var dropdown_xy = block.getFieldValue('xy');
    var number_name = block.getFieldValue('NAME');
    var code = 'set_default_xy('+ dropdown_xy + ',' + number_name + ')\n';
    return code;
  },
  javascriptGenerator: (block, js) => {
    return `simulator.dummy();`;
  }
}