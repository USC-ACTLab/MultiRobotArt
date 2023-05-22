import { RobartBlockDefinition } from '../BlockDefinition';
import Blockly from 'blockly';

export const block_takeoff: RobartBlockDefinition = {
  name: "takeoff",
  block:{
    init: function() {
      this.appendDummyInput()
      .appendField("takeoff to")
      .appendField(new Blockly.FieldNumber(0), "height")
      .appendField("meters at")
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
    var code = 'takeoff(' + number_height + ',' + number_speed + ')\n';
    return code;
  },
  javascriptGenerator: (block, js) => {
    return `simulator.dummy();`;
  }
}