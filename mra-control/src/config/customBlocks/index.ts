import {pythonGenerator} from 'blockly/python';
import {javascriptGenerator as js} from 'blockly/javascript';
import {blockCircle} from './curves/circle';
import {blockStop} from './motion/stop';
import {blockGoTo} from './motion/go_to_duration';
import {blockGoToSpeed} from './motion/go_to';
import {blockLand} from './motion/land';
import {blockMoveAngles} from './motion/move_angles';
import {blockMoveXyz} from './motion/move_xyz';
import {blockMove} from './motion/move';
import {blockTakeoff} from './motion/takeoff';
import {blockTurn} from './motion/turn';
import {blockGetPosition} from './utility/get_a_position';
import {blockColor} from './colors/color';
import {blockColorOff} from './colors/colorOff';
import Blockly from 'blockly';
import {multiTraj} from './motion/multi_traj';
import {blockCircleArc} from './curves/circleArc';
import {blockNegate} from './trajectoryModifiers/negate_trajectory';


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
	blockStop,
	blockTakeoff,
	blockTurn,
	blockCircle,
	blockCircleArc,
	blockGetPosition,
	blockColor,
	blockColorOff,
	multiTraj,
	blockNegate,
};

Object.values(CUSTOM_BLOCKS).forEach((block) => {
	Blockly.Blocks[block.name] = block.block;
	pythonGenerator[block.name] = (b: Blockly.Block) => block.pythonGenerator(b, pythonGenerator);
	js[block.name] = (b: Blockly.Block) => block.javascriptGenerator(b, js);
});
