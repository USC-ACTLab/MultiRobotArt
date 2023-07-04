import { RobartBlockDefinition } from '../BlockDefinition';
import Blockly from 'blockly';

export const block_takeoff: RobartBlockDefinition = {
  name: "takeoff",
  block:{
    init: function() {
      this.appendDummyInput()
      .appendField("takeoff to")
      .appendField(new Blockly.FieldNumber(0), "height")
      .appendField("meters over")
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
    var code = 'takeoff(cf, ' + number_height + ' ,' + duration + ')\n';
    return code;
  },
  javascriptGenerator: (block, js) => {
    var number_height = block.getFieldValue('height');
    var duration = block.getFieldValue('duration');
    return `duration += simulator.takeoff(group_state, ${number_height}, ${duration});\n`;
  }
}