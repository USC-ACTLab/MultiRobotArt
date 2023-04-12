import { ToolboxDefinition } from 'react-blockly';

export const blocklyToolboxConfiguration: ToolboxDefinition = {
  kind: 'categoryToolbox',
  contents: [
    {
      kind: 'category',
      name: 'Motion',
      contents: [
        {
          kind: 'block',
          type: 'go_to_xyz',
        },
      ],
    },
    {
      kind: 'category',
      name: 'Config',
      contents: [
        {
          kind: 'block',
          type: 'set_default_height',
        },
      ],
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
