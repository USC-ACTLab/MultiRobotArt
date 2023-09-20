import { pythonGenerator } from 'blockly/python';
import { javascriptGenerator as js } from 'blockly/javascript';
import Blockly from 'blockly';
// import { block_go_to_xyz } from './motion/go_to_xyz';
import { block_circle } from './curves/circle';
import { block_start_linear_motion } from './motion/start_linear_motion';
import { block_set_default_height } from './utility/set_default_height';
import { block_stop } from './motion/stop';
import { block_go_to } from './motion/go_to_duration';
import { block_go_to_speed } from './motion/go_to'
import { block_land } from './motion/land';
import { block_move_angles } from './motion/move_angles';
import { block_move_xyz } from './motion/move_xyz';
import { block_move } from './motion/move';
import { block_takeoff } from './motion/takeoff';
import { block_turn } from './motion/turn';
import { block_start_circle } from './curves/start_circle';
import { block_start_move } from './curves/start_move';
import { block_start_turn } from './curves/start_turn';
import { block_get_a_position } from './utility/get_a_position';
import { block_set_default_speed } from './utility/set_default_speed';
import { block_set_default_xy } from './utility/set_default_xy';
import { block_color } from './colors/color'


/**
 * This is where we collect all of the custom blocks and actually update Blockly definitions.
 */
export const CUSTOM_BLOCKS = {
    //  block_go_to_xyz,
     block_go_to_speed,
     block_go_to,
     block_land,
     block_move_angles,
     block_move_xyz,
     block_move,
     block_start_linear_motion,
     block_stop,
     block_takeoff,
     block_turn,
     block_circle,
     block_start_circle,
     block_start_move,
     block_start_turn,
     block_get_a_position,
     block_set_default_height,
     block_set_default_speed,
     block_set_default_xy,
     block_color
};

Object.values(CUSTOM_BLOCKS).forEach((block) => {
    Blockly.Blocks[block.name] = block.block;
    pythonGenerator[block.name] = (b: Blockly.Block) => block.pythonGenerator(b, pythonGenerator);
    js[block.name] = (b: Blockly.Block) => block.javascriptGenerator(b, js);
});
