
import Blockly from 'blockly';

export type BlocklyOrder = 'ORDER_ATOMIC';

export interface BlocklyGenerator {
    valueToCode: (block: Blockly.Block, name: string, order: BlocklyOrder) => any;
    ORDER_ATOMIC: BlocklyOrder;
}

/**
 * A lightweight wrapping interface for defining custom blocks within Robart.
 */
export interface RobartBlockDefinition {
    /**
     * The name of the block to define.
     */
    name: string;
    /**
     * The actual Blockly block definition (ex. what is exported from the Blockly editor).
     */
    block: any | {
        init: () => void;
    }; 
    /**
     * Function to generate Python code for this block.
     * @param block The blockly block to generate code for.
     * @param python An instance of the Blockly Python generator.
     * @returns A string of Python code.
     */
    pythonGenerator: (block: Blockly.Block, python: BlocklyGenerator) => string;
    /**
     * Function to generate JavaScript code for this block.
     * @param block The blockly block to generate code for.
     * @param js An instance of the Blockly JavaScript generator.
     * @returns A string of JavaScript code.
     */
    javascriptGenerator: (block: Blockly.Block, js: BlocklyGenerator) => string;
}