import { RobartBlockDefinition } from '../BlockDefinition';
import Blockly from 'blockly';

export const block_start_move: RobartBlockDefinition = {
  name: "start_move",
  block:{
    init: function() {
      this.appendDummyInput()
        .appendField("start moving")
        .appendField(new Blockly.FieldDropdown([["up","up"], ["down","down"], ["left","left"], ["right","right"], ["forward","forward"], ["backward","backward"]]), "direction")
        .appendField("at")
        .appendField(new Blockly.FieldNumber(0), "speed")
        .appendField("m/s");
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(90);
      this.setTooltip("");
      this.setHelpUrl("");
    }
  },
  pythonGenerator: (block, python) => {
    var dropdown_direction = block.getFieldValue('direction');
    var number_speed = block.getFieldValue('speed');
    var code = 'start_move(' + dropdown_direction + ',' + number_speed + ')\n';
    return code;
  },
  javascriptGenerator: (block, js) => {
    return `duration += simulator.dummy();`;
  }
}