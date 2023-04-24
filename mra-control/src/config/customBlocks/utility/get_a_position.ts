import { RobartBlockDefinition } from '../BlockDefinition';
import Blockly from 'blockly';

export const block_get_a_position: RobartBlockDefinition = {
  name: "get_a_position",
  block: {
    init: function () {
        this.appendDummyInput()
        .appendField(new Blockly.FieldDropdown([["default","default"], ["current","current"]]), "which")
        .appendField(new Blockly.FieldDropdown([["x","x position"], ["y","y position"], ["z","z position"]]), "NAME")
        .appendField("position");
    this.setOutput(true, null);
    this.setColour(230);
 this.setTooltip("");
 this.setHelpUrl("");
    },
  },
  pythonGenerator: (block, python) => {
    var dropdown_which = block.getFieldValue('which');
    var dropdown_name = block.getFieldValue('NAME');
    var code = 'get_a_position(' + dropdown_which + ',' + dropdown_name + ')';
    // TODO: Change ORDER_NONE to the correct strength.
    return code;
  },
  javascriptGenerator: (block, js) => {
    return `simulator.dummy()`;
  }
}