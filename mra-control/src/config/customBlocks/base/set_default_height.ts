import Blockly from "blockly";
import { pythonGenerator } from "blockly/python";

Blockly.Blocks["set_default_height"] = {
  init: function () {
    this.appendDummyInput()
      .appendField("set default height to")
      .appendField(new Blockly.FieldNumber(0, 0), "default_height")
      .appendField("meters");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
    this.setTooltip("");
    this.setHelpUrl("");
  },
};

pythonGenerator["set_default_height"] = (block: Blockly.Block) => {
  var number_default_height = block.getFieldValue("default_height");
  var code = `setDefaultHeight(${number_default_height})\n`;
  return code;
};
