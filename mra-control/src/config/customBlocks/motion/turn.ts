import { RobartBlockDefinition } from '../BlockDefinition';
import Blockly from 'blockly';

export const block_turn: RobartBlockDefinition = {
  name: "turn",
  block:{
    init: function() {
        this.appendDummyInput()
        .appendField("turn")
        .appendField(new Blockly.FieldDropdown([["right","\n\t\t\t\t\t\t\t\t\tright\n\t\t\t\t\t\t\t\t"], ["left","left"]]), "direction")
        .appendField(new Blockly.FieldAngle(0), "degrees")
        .appendField("at")
        .appendField(new Blockly.FieldAngle(0), "rate")
        .appendField("degrees/s");
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(90);
        this.setTooltip("");
        this.setHelpUrl("");
    }
  },
  pythonGenerator: (block, python) => {
    //TODO: add python equivalent
    var dropdown_direction = block.getFieldValue('direction');
    var angle_degrees = block.getFieldValue('degrees');
    var angle_rate = block.getFieldValue('rate');
    var code = 'turn(' + dropdown_direction + ',' + angle_degrees + ',' + angle_rate + ')\n';
    return code;
  },
  javascriptGenerator: (block, js) => {
    return `duration += simulator.dummy();`;
  }
}