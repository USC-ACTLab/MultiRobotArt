Blockly.common.defineBlocksWithJsonArray([
	{
  "type": "set_default_height",
  "message0": "set default height to %1 meters",
  "args0": [
    {
      "type": "field_number",
      "name": "default_height",
      "value": 0,
      "min": 0
    }
  ],
  "previousStatement": null,
  "nextStatement": null,
  "colour": 230,
  "tooltip": "",
  "helpUrl": ""
},
{
  "type": "go_to",
  "message0": "go to X: %1 Y: %2 Z: %3 at %4 m/s",
  "args0": [
    {
      "type": "field_number",
      "name": "x_pos",
      "value": 0
    },
    {
      "type": "field_number",
      "name": "y_pos",
      "value": 0
    },
    {
      "type": "field_number",
      "name": "z_pos",
      "value": 0
    },
    {
      "type": "field_number",
      "name": "speed",
      "value": 0
    }
  ],
  "inputsInline": false,
  "previousStatement": null,
  "nextStatement": null,
  "colour": 90,
  "tooltip": "",
  "helpUrl": ""
},
{
  "type": "get_a_position",
  "message0": "%1 %2 position",
  "args0": [
    {
      "type": "field_dropdown",
      "name": "which",
      "options": [
        [
          "default",
          "default"
        ],
        [
          "current",
          "current"
        ]
      ]
    },
    {
      "type": "field_dropdown",
      "name": "NAME",
      "options": [
        [
          "x",
          "x position"
        ],
        [
          "y",
          "y position"
        ],
        [
          "z",
          "z position"
        ]
      ]
    }
  ],
  "output": null,
  "colour": 230,
  "tooltip": "",
  "helpUrl": ""
},
{
  "type": "move",
  "message0": "move %1 %2 meters at %3 m/s",
  "args0": [
    {
      "type": "field_dropdown",
      "name": "direction",
      "options": [
        [
          "up",
          "up"
        ],
        [
          "down",
          "down"
        ],
        [
          "left",
          "left"
        ],
        [
          "right",
          "right"
        ],
        [
          "forward",
          "forward"
        ],
        [
          "backward",
          "backward"
        ]
      ]
    },
    {
      "type": "field_number",
      "name": "distance",
      "value": 0
    },
    {
      "type": "field_number",
      "name": "NAME",
      "value": 0
    }
  ],
  "previousStatement": null,
  "nextStatement": null,
  "colour": 90,
  "tooltip": "",
  "helpUrl": ""
},
{
  "type": "move_xyz",
  "message0": "move X %1 meters %2 move Y %3 meters %4 move Z %5 meters %6 at %7 m/s",
  "args0": [
    {
      "type": "field_number",
      "name": "x",
      "value": 0
    },
    {
      "type": "input_dummy"
    },
    {
      "type": "field_number",
      "name": "y",
      "value": 0
    },
    {
      "type": "input_dummy"
    },
    {
      "type": "field_number",
      "name": "z",
      "value": 0
    },
    {
      "type": "input_dummy"
    },
    {
      "type": "field_number",
      "name": "speed",
      "value": 0
    }
  ],
  "previousStatement": null,
  "nextStatement": null,
  "colour": 90,
  "tooltip": "",
  "helpUrl": ""
},
{
  "type": "move_by_angles",
  "message0": "move towards %1 horizontal angle %2 %3 vertical angle %4 %5 distance %6 %7 at %8 m/s",
  "args0": [
    {
      "type": "input_dummy"
    },
    {
      "type": "field_angle",
      "name": "horizontal angle",
      "angle": 90
    },
    {
      "type": "input_dummy"
    },
    {
      "type": "field_angle",
      "name": "vertical angle",
      "angle": 90
    },
    {
      "type": "input_dummy"
    },
    {
      "type": "field_number",
      "name": "distance",
      "value": 0
    },
    {
      "type": "input_dummy"
    },
    {
      "type": "field_number",
      "name": "NAME",
      "value": 0
    }
  ],
  "colour": 90,
  "tooltip": "",
  "helpUrl": ""
},
{
  "type": "set_default_speed",
  "message0": "set default speed to %1",
  "args0": [
    {
      "type": "field_number",
      "name": "speed",
      "value": 0
    }
  ],
  "previousStatement": null,
  "nextStatement": null,
  "colour": 230,
  "tooltip": "",
  "helpUrl": ""
},
{
  "type": "set_default_xy",
  "message0": "set default %1 to %2",
  "args0": [
    {
      "type": "field_dropdown",
      "name": "xy",
      "options": [
        [
          "x",
          "x"
        ],
        [
          "y",
          "y"
        ]
      ]
    },
    {
      "type": "field_number",
      "name": "NAME",
      "value": 0
    }
  ],
  "previousStatement": null,
  "nextStatement": null,
  "colour": 230,
  "tooltip": "",
  "helpUrl": ""
},
{
  "type": "takeoff",
  "message0": "takeoff to %1 meters at %2 m/s",
  "args0": [
    {
      "type": "field_number",
      "name": "height",
      "value": 0
    },
    {
      "type": "field_number",
      "name": "speed",
      "value": 0
    }
  ],
  "previousStatement": null,
  "nextStatement": null,
  "colour": 230,
  "tooltip": "",
  "helpUrl": ""
},
{
  "type": "land",
  "message0": "land on %1 meter high object at %2 m/s",
  "args0": [
    {
      "type": "field_number",
      "name": "height",
      "value": 0
    },
    {
      "type": "field_number",
      "name": "speed",
      "value": 0
    }
  ],
  "previousStatement": null,
  "nextStatement": null,
  "colour": 230,
  "tooltip": "",
  "helpUrl": ""
},
{
  "type": "get_position",
  "message0": "get position",
  "output": null,
  "colour": 230,
  "tooltip": "",
  "helpUrl": ""
}]);
