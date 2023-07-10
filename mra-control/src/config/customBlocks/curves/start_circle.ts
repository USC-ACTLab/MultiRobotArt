import { RobartBlockDefinition } from '../BlockDefinition';
import Blockly from 'blockly';

export const block_start_circle: RobartBlockDefinition = {
  name: "start_circle",
  block:{
    init: function() {
      this.appendDummyInput()
      .appendField("start cricling")
      .appendField(new Blockly.FieldNumber(0), "radius")
      .appendField("meter radius")
      .appendField(new Blockly.FieldDropdown([["right","circle_right"], ["left","circle_left"]]), "direction");
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(90);
      this.setTooltip("");
      this.setHelpUrl("");
    }
  },
  pythonGenerator: (block, python) => {
    var number_radius = block.getFieldValue('radius');
    var dropdown_direction = block.getFieldValue('direction');
    var code = 'start_circle(' + number_radius + ',' + dropdown_direction + ')\n';
    return code;
  },
  javascriptGenerator: (block, js) => {
    return `duration += simulator.dummy();`;
  }
}