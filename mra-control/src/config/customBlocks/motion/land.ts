import { RobartBlockDefinition } from '../BlockDefinition';
import Blockly from 'blockly';

export const block_land: RobartBlockDefinition = {
  name: "land",
  block:{
    init: function() {
    this.appendDummyInput()
        .appendField("land at height")
        .appendField(new Blockly.FieldNumber(0), "height")
        .appendField("over")
        .appendField(new Blockly.FieldNumber(0), "duration")
        .appendField("seconds");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
  this.setTooltip("");
  this.setHelpUrl("");
    }
  },
  pythonGenerator: (block, python) => {
    var number_height = block.getFieldValue('height');
    var duration = block.getFieldValue('duration');
    var code = 'land(cf, ' + number_height + ',' + duration + ')\n';
    return code;
  },
  javascriptGenerator: (block, js) => {
    var number_height = block.getFieldValue('height');
    var duration = block.getFieldValue('duration');
    return `duration += simulator.land(group_state, ${number_height}, ${duration});\n`;
  }
}