import {pythonGenerator} from 'blockly/python';
import {javascriptGenerator as js} from 'blockly/javascript';
import {blockCircle} from './curves/circle';
import {blockStartLinearMotion} from './motion/start_linear_motion';
import {blockStop} from './motion/stop';
import {blockGoTo} from './motion/go_to_duration';
import {blockGoToSpeed} from './motion/go_to';
import {blockLand} from './motion/land';
import {blockMoveAngles} from './motion/move_angles';
import {blockMoveXyz} from './motion/move_xyz';
import {blockMove} from './motion/move';
import {blockTakeoff} from './motion/takeoff';
import {blockTurn} from './motion/turn';
import {block_start_circle} from './curves/start_circle';
import {block_start_move} from './curves/start_move';
import {block_start_turn} from './curves/start_turn';
import {blockGetPosition} from './utility/get_a_position';
import {blockColor} from './colors/color';
import Blockly from 'blockly';
import {multiTraj} from './motion/multi_traj';


/**
 * This is where we collect all of the custom blocks and actually update Blockly definitions.
 */
export const CUSTOM_BLOCKS = {
	//  blockGoTo_xyz,
	blockGoToSpeed,
	blockGoTo,
	blockLand,
	blockMoveAngles,
	blockMoveXyz,
	blockMove,
	blockStartLinearMotion,
	blockStop,
	blockTakeoff,
	blockTurn,
	blockCircle,
	block_start_circle,
	block_start_move,
	block_start_turn,
	blockGetPosition,
	blockColor,
	multiTraj,
};

Object.values(CUSTOM_BLOCKS).forEach((block) => {
	Blockly.Blocks[block.name] = block.block;
	pythonGenerator[block.name] = (b: Blockly.Block) => block.pythonGenerator(b, pythonGenerator);
	js[block.name] = (b: Blockly.Block) => block.javascriptGenerator(b, js);
	console.log(block.name);
});
