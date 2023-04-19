import { RobartBlockDefinition } from '../BlockDefinition';
import Blockly from 'blockly';

export const block_land: RobartBlockDefinition = {
  name: "land",
  block:{
    init: function() {
    this.appendDummyInput()
        .appendField("land on")
        .appendField(new Blockly.FieldNumber(0), "height")
        .appendField("meter high object")
        .appendField("at")
        .appendField(new Blockly.FieldNumber(0), "speed")
        .appendField("m/s");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
  this.setTooltip("");
  this.setHelpUrl("");
    }
  },
  pythonGenerator: (block, python) => {
    var number_height = block.getFieldValue('height');
    var number_speed = block.getFieldValue('speed');
    var code = 'land(' + number_height + ',' + number_speed + ')\n';
    return code;
  },
  javascriptGenerator: (block, js) => {
    return `simulator.dummy();`;
  }
}