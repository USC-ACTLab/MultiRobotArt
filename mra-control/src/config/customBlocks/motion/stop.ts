import { RobartBlockDefinition } from '../BlockDefinition';
import Blockly from 'blockly';

export const block_stop: RobartBlockDefinition = {
  name: "stop",
  block:{
    init: function() {
      this.appendDummyInput()
        .appendField("stop and hover");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(0);
 this.setTooltip("");
 this.setHelpUrl("");
    }
  },
  pythonGenerator: (block, python) => {
    return 'stop_and_hover(cf)\n';
  },
  javascriptGenerator: (block, js) => {
    return `simulator.dummy();`;
  }
}