import {pythonGenerator} from 'blockly/python';
import {javascriptGenerator as js} from 'blockly/javascript';
import {blockCircle} from './curves/circle';
import {blockCircleRadians} from './curves/circle';
import {blockStop} from './motion/stop';
import {blockGoTo} from './motion/go_to_duration';
import {blockGoToSpeed} from './motion/go_to';
import {blockLand} from './motion/land';
import {blockMoveXyz} from './motion/move_xyz';
import {blockMove} from './motion/move';
import {blockTakeoff} from './motion/takeoff';
import {blockGetPosition} from './utility/get_a_position';
import {blockColor, blockRandomColor} from './colors/color';
import {blockColorOff} from './colors/colorOff';
import Blockly from 'blockly';
import {multiTraj} from './motion/multi_traj';
import {blockCircleArc} from './curves/circleArc';
import {blockNegate} from './trajectoryModifiers/negate_trajectory';
import {blockAddTrajectories} from './trajectoryModifiers/add_trajectories';
import {blockSubtractTrajectories} from './trajectoryModifiers/subtract_trajectories';
import {blockStretchTrajectories} from './trajectoryModifiers/stretch_trajectory';
import {blockRotateTrajectoryDegrees, blockRotateTrajectoryRadians} from './trajectoryModifiers/rotate_trajectory';

/**
 * This is where we collect all of the custom blocks and actually update Blockly definitions.
 */
export const CUSTOM_BLOCKS = {
	//  blockGoTo_xyz,
	blockGoToSpeed,
	blockGoTo,
	blockLand,
	blockMoveXyz,
	// blockMove,
	blockStop,
	blockTakeoff,
	blockCircle,
	blockCircleRadians,
	blockCircleArc,
	blockGetPosition,
	blockColor,
	blockColorOff,
	multiTraj,
	blockNegate,
	blockAddTrajectories,
	blockSubtractTrajectories,
	blockStretchTrajectories,
	blockRotateTrajectoryDegrees,
	blockRotateTrajectoryRadians,
	blockRandomColor,
};

Object.values(CUSTOM_BLOCKS).forEach((block) => {
	Blockly.Blocks[block.name] = block.block;
	pythonGenerator[block.name] = (b: Blockly.Block) => block.pythonGenerator(b, pythonGenerator);
	js[block.name] = (b: Blockly.Block) => block.javascriptGenerator(b, js);
});
