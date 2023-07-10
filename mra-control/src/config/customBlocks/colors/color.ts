import { RobartBlockDefinition } from '../BlockDefinition';
import Blockly from 'blockly';
import {ColorWheelField} from './ColorWheel';
import { Color } from 'three';
import { string } from 'blockly/core/utils';

// Blockly.Blocks["color_wheel_picker"] = {
//   init: function () {
//     this.appendDummyInput()
//       .appendField("Color: ")
//       .appendField(new ColorPickerField("#00FF00", 150, {
//         layoutDirection: 'horizontal',
//       }), "COLOR")
//   }
// };
function hexToRgb(hex: string) {
  var result = /^#?([a-f\d]{2})([a-f\d]{2})([a-f\d]{2})$/i.exec(hex);
  return result ? {
    r: parseInt(result[1], 16),
    g: parseInt(result[2], 16),
    b: parseInt(result[3], 16)
  } : null;
};

export const block_color: RobartBlockDefinition = {
  name: "color",
  block:{
    init: function() {
      this.appendDummyInput()
        .appendField('set LED color:')
        .appendField(new ColorWheelField("#00FF00", 150, {
            layoutDirection: 'horizontal',
          }), "color")
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
    }
    
  },

  pythonGenerator: (block, python) => {
    const color = block.getFieldValue('color');
    const code = 'setLEDColorFromHex(cf, \"' + color + '\")\n';
    return code;
  },



  javascriptGenerator: (block, js) => {
    var color = block.getFieldValue('color');
    var rgbCol = hexToRgb(color);
    return `duration += simulator.setColor(group_state, ${rgbCol?.r},${rgbCol?.g},${rgbCol?.b});`;

  }
}
