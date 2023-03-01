import Blockly from "blockly";
import { pythonGenerator as python } from "blockly/python";

Blockly.Blocks["go_to_xyz"] = {
  init: function () {
    this.appendDummyInput().appendField("go to");
    this.appendValueInput("x").setCheck("Number").appendField("X:");
    this.appendValueInput("y").setCheck("Number").appendField("Y:");
    this.appendValueInput("z").setCheck("Number").appendField("Z:");
    this.appendValueInput("speed")
      .setCheck("Number")
      .appendField("at speed (m/s)");
    this.setInputsInline(false);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(90);
    this.setTooltip("");
    this.setHelpUrl("");
  },
};

python["go_to_xyz"] = (block: Blockly.Block) => {
  var x = python.valueToCode(block, "x", python.ORDER_ATOMIC);
  var y = python.valueToCode(block, "y", python.ORDER_ATOMIC);
  var z = python.valueToCode(block, "z", python.ORDER_ATOMIC);
  var speed = python.valueToCode(block, "speed", python.ORDER_ATOMIC);
  var code = `goTo(${x}, ${y}, ${z}, ${speed})\n`;
  return code;
};
