import {type ToolboxDefinition} from 'react-blockly';
import {type RobartBlockDefinition} from './customBlocks/BlockDefinition';
import {CUSTOM_BLOCKS} from './customBlocks';

const blockToToolbox = (block: RobartBlockDefinition) => ({
	kind: 'block',
	type: block.name,
});

export const blocklyToolboxConfiguration: ToolboxDefinition = {
	kind: 'categoryToolbox',
	contents: [
		{
			kind: 'category',
			name: 'Straight Motion',
			contents: [
				// blockToToolbox(CUSTOM_BLOCKS.blockGoTo_xyz),
				blockToToolbox(CUSTOM_BLOCKS.blockTakeoff),
				blockToToolbox(CUSTOM_BLOCKS.blockLand),
				blockToToolbox(CUSTOM_BLOCKS.blockGoTo),
				blockToToolbox(CUSTOM_BLOCKS.blockGoToSpeed),
				blockToToolbox(CUSTOM_BLOCKS.blockMoveXyz),
				blockToToolbox(CUSTOM_BLOCKS.blockMove),
				//blockToToolbox(CUSTOM_BLOCKS.blockStartLinearMotion),
				blockToToolbox(CUSTOM_BLOCKS.blockStop),
				//blockToToolbox(CUSTOM_BLOCKS.blockTurn),
			],
		},
		{
			kind: 'category',
			name: 'Curves',
			contents: [
				blockToToolbox(CUSTOM_BLOCKS.blockCircle),
				blockToToolbox(CUSTOM_BLOCKS.blockCircleArc),
				//blockToToolbox(CUSTOM_BLOCKS.block_start_circle),
			],
		},
		{
			kind: 'category',
			name: 'Config',
			contents: [
				blockToToolbox(CUSTOM_BLOCKS.blockGetPosition),
			],
		},
		{
			kind: 'category',
			name: 'Colors',
			contents: [
				blockToToolbox(CUSTOM_BLOCKS.blockColor),
				blockToToolbox(CUSTOM_BLOCKS.blockRandomColor),
				blockToToolbox(CUSTOM_BLOCKS.blockColorOff),
			],
		},
		{
			kind: 'category',
			name: 'Trajectory Modifiers',
			contents: [
				blockToToolbox(CUSTOM_BLOCKS.blockNegate),
				blockToToolbox(CUSTOM_BLOCKS.multiTraj),
				blockToToolbox(CUSTOM_BLOCKS.blockAddTrajectories),
				blockToToolbox(CUSTOM_BLOCKS.blockSubtractTrajectories),
				blockToToolbox(CUSTOM_BLOCKS.blockStretchTrajectories),
				blockToToolbox(CUSTOM_BLOCKS.blockRotateTrajectoryDegrees),
				blockToToolbox(CUSTOM_BLOCKS.blockRotateTrajectoryRadians),
			],
		},
		// {
		// 	kind: 'category',
		// 	name: 'Logic',
		// 	// colour: '#5C81A6',
		// 	contents: [
		// 		{
		// 			kind: 'block',
		// 			type: 'controls_if',
		// 		},
		// 		{
		// 			kind: 'block',
		// 			type: 'logic_compare',
		// 		},
		// 	],
		// },
		// {
		// 	kind: 'category',
		// 	name: 'Math',
		// 	// colour: '#5CA65C',
		// 	contents: [
		// 		{
		// 			kind: 'block',
		// 			type: 'math_round',
		// 		},
		// 		{
		// 			kind: 'block',
		// 			type: 'math_number',
		// 		},
		// 	],
		// },
	],
};
