import Blockly from 'blockly';
import { pythonGenerator as python } from 'blockly/python';
import { javascriptGenerator as js } from 'blockly/javascript';
import { useSimulator } from '@MRAControl/state/useSimulator';

Blockly.Blocks['go_to_xyz'] = {
  init: function () {
    this.appendDummyInput().appendField('go to');
    this.appendValueInput('x').setCheck('Number').appendField('X:');
    this.appendValueInput('y').setCheck('Number').appendField('Y:');
    this.appendValueInput('z').setCheck('Number').appendField('Z:');
    this.appendValueInput('speed').setCheck('Number').appendField('at speed (m/s)');
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(90);
    this.setTooltip('');
    this.setHelpUrl('');
  },
};

python['go_to_xyz'] = (block: Blockly.Block) => {
  var x = python.valueToCode(block, 'x', python.ORDER_ATOMIC);
  var y = python.valueToCode(block, 'y', python.ORDER_ATOMIC);
  var z = python.valueToCode(block, 'z', python.ORDER_ATOMIC);
  var speed = python.valueToCode(block, 'speed', python.ORDER_ATOMIC);
  var code = `goTo(${x}, ${y}, ${z}, ${speed})\n`;
  return code;
};

js['go_to_xyz'] = (block: Blockly.Block) => {
  var x = js.valueToCode(block, 'x', js.ORDER_ATOMIC);
  var y = js.valueToCode(block, 'y', js.ORDER_ATOMIC);
  var z = js.valueToCode(block, 'z', js.ORDER_ATOMIC);
  var speed = js.valueToCode(block, 'speed', js.ORDER_ATOMIC);

  return `simulator_go_to_xyz(group_state, ${x}, ${y}, ${z}, ${speed})`;
};
