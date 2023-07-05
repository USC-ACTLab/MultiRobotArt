import { ToolboxDefinition } from 'react-blockly';
import { RobartBlockDefinition } from './customBlocks/BlockDefinition';
import { CUSTOM_BLOCKS } from './customBlocks';

const blockToToolbox = (block: RobartBlockDefinition) => ({
  kind: 'block',
  type: block.name,
});

export const blocklyToolboxConfiguration: ToolboxDefinition = {
  kind: 'categoryToolbox',
  contents: [
    {
      kind: 'category',
      name: 'Motion',
      contents: [
        // blockToToolbox(CUSTOM_BLOCKS.block_go_to_xyz),
        blockToToolbox(CUSTOM_BLOCKS.block_go_to),
        blockToToolbox(CUSTOM_BLOCKS.block_go_to_speed),
        blockToToolbox(CUSTOM_BLOCKS.block_takeoff),
        blockToToolbox(CUSTOM_BLOCKS.block_land),
        blockToToolbox(CUSTOM_BLOCKS.block_move_angles),
        blockToToolbox(CUSTOM_BLOCKS.block_move_xyz),
        blockToToolbox(CUSTOM_BLOCKS.block_move),
        blockToToolbox(CUSTOM_BLOCKS.block_start_linear_motion),
        blockToToolbox(CUSTOM_BLOCKS.block_stop),
        blockToToolbox(CUSTOM_BLOCKS.block_turn),
      ],
    },
    {
      kind: 'category',
      name: 'Curves',
      contents: [
        blockToToolbox(CUSTOM_BLOCKS.block_circle),
        blockToToolbox(CUSTOM_BLOCKS.block_start_circle),
        blockToToolbox(CUSTOM_BLOCKS.block_start_move),
        blockToToolbox(CUSTOM_BLOCKS.block_start_turn),
      ],
    },
    {
      kind: 'category',
      name: 'Config',
      contents: [
        blockToToolbox(CUSTOM_BLOCKS.block_get_a_position),
        blockToToolbox(CUSTOM_BLOCKS.block_set_default_height),
        blockToToolbox(CUSTOM_BLOCKS.block_set_default_speed),
        blockToToolbox(CUSTOM_BLOCKS.block_set_default_xy)
      ],
    },
    {
      kind: 'category',
      name: 'Colors',
      contents: [
        blockToToolbox(CUSTOM_BLOCKS.block_color)
      ]
    },
    {
      kind: 'category',
      name: 'Logic',
      colour: '#5C81A6',
      contents: [
        {
          kind: 'block',
          type: 'controls_if',
        },
        {
          kind: 'block',
          type: 'logic_compare',
        },
      ],
    },
    {
      kind: 'category',
      name: 'Math',
      colour: '#5CA65C',
      contents: [
        {
          kind: 'block',
          type: 'math_round',
        },
        {
          kind: 'block',
          type: 'math_number',
        },
      ],
    },
    {
      kind: 'category',
      name: 'Custom',
      colour: '#5CA699',
      contents: [
        {
          kind: 'block',
          type: 'new_boundary_function',
        },
        {
          kind: 'block',
          type: 'return',
        },
      ],
    },
  ],
};
