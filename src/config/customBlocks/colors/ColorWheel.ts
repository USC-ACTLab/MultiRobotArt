import * as Blockly from 'blockly/core';
import iro from '@jaames/iro';
import type {ColorPickerProps} from '@jaames/iro/dist/ColorPicker';

/**
 * This is the class for the color wheel.
 */
export class ColorWheelField extends Blockly.FieldColour {
  /**
   * Class for the color picker.
   *
   * @param {string} color The starting color for the color.
   *  It's a hex value, #ff00ff.
   * @param {number} width Width of the color picker.
   * @param {ColorPickerProps} options The iro color wheel options.
   */
  constructor(
    protected color: string,
    protected width = 150,
    protected options: Partial<ColorPickerProps> = {}
  ) {
    super(color);
  }
  /**
   * Constructs a ColorWheelField from a JSON arg object.
   * @param {!Object} options A JSON object with options.
   * @return {!ColorWheelField} The new field instance.
   * @package
   * @nocollapse
   */
  static fromJson(options: { [x: string]: any; }) {
    return new ColorWheelField(
        options['color'],
        options['size'] || 150,
        options['options'] || {}
    );
  }

  /**
   * Over rides colour picker to show the popup.
   */
  showEditor_() {
    const editor = document.createElement('div');
    // Appends to the content div
    Blockly.DropDownDiv.getContentDiv().appendChild(editor);
    // Add class so it can be styled easily
    editor.classList.add('blockly-color-wheel-container');
    // Will position the content.  The last argument is a
    // callback that used for cleanup.

    // eslint-disable-next-line new-cap
    const colorPicker = iro.ColorPicker(editor, {
      width: this.width, // controls the size of the color picker
      color: this.getValue(), // starts the color picker at a certain value,
      ...this.options, // These options will over ride everything
    });

    // Callback when the color picker changes
    colorPicker.on('color:change', (color: { hexString: any; }) => {
      this.setValue(color.hexString);
    });
    Blockly.DropDownDiv.showPositionedByField(this, () => editor.remove());
  }
}

Blockly.fieldRegistry.register('color_wheel', ColorWheelField);