import { RobartBlockDefinition } from '../BlockDefinition';
import Blockly from 'blockly';
import {ColorWheelField} from './ColorWheel';

// Blockly.Blocks["color_wheel_picker"] = {
//   init: function () {
//     this.appendDummyInput()
//       .appendField("Color: ")
//       .appendField(new ColorPickerField("#00FF00", 150, {
//         layoutDirection: 'horizontal',
//       }), "COLOR")
//   }
// };

export const block_color: RobartBlockDefinition = {
  name: "color",
  block:{
    init: function() {
      this.appendDummyInput()
        .appendField('set LED color:')
        .appendField(new ColorWheelField("#00FF00", 150, {
            layoutDirection: 'horizontal',
          }), "COLOR")
    }
  },

  pythonGenerator: (block, python) => {
    var code = '';
    return code;
  },
  javascriptGenerator: (block, js) => {
    return `simulator.dummy();`;
  }
}