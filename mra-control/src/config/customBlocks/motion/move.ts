import { RobartBlockDefinition } from '../BlockDefinition';
import Blockly from 'blockly';

export const block_move: RobartBlockDefinition = {
  name: "move",
  block: {
    init: function () {
        this.appendDummyInput()
        .appendField("move")
        .appendField(new Blockly.FieldDropdown([["up","up"], ["down","down"], ["left","left"], ["right","right"], ["forward","forward"], ["backward","backward"]]), "direction")
        .appendField(new Blockly.FieldNumber(0), "distance")
        .appendField("meters for")
        .appendField(new Blockly.FieldNumber(0), "duration")
        .appendField("seconds");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(90);
 this.setTooltip("");
 this.setHelpUrl("");
    },
  },
  pythonGenerator: (block, python) => {
    var dropdown_direction = block.getFieldValue('direction');
    var number_distance = block.getFieldValue('distance');
    var duration = block.getFieldValue('duration');
    var code = 'move_direction(cf, ' + dropdown_direction + ',' + number_distance + ',' + duration + ')\n';
    return code;
  },
  javascriptGenerator: (block, js) => {
    return `simulator.dummy()`;
  }
}