[{
  "type": "circle",
  "message0": "%1 %2 m %3 at speed %4 m/s %5 for %6 degrees, %7 %8",
  "args0": [
    {
      "type": "field_label_serializable",
      "name": "Circular motion",
      "text": "Go in circle of radius"
    },
    {
      "type": "field_number",
      "name": "radius_m",
      "value": 0,
      "min": 0
    },
    {
      "type": "input_dummy"
    },
    {
      "type": "field_number",
      "name": "velocity",
      "value": 0.2
    },
    {
      "type": "input_dummy"
    },
    {
      "type": "field_angle",
      "name": "angle_degrees",
      "angle": 90
    },
    {
      "type": "input_dummy"
    },
    {
      "type": "field_dropdown",
      "name": "direction",
      "options": [
        [
          "counterclockwise",
          "ccw"
        ],
        [
          "clockwise",
          "cw"
        ]
      ]
    }
  ],
  "previousStatement": null,
  "nextStatement": null,
  "colour": 90,
  "tooltip": "",
  "helpUrl": ""
},
{
  "type": "start_linear_motion",
  "message0": "Start linear motion with speeds in %1 x direction: %2 m/s %3 y direction: %4 m/s %5 z direction: %6 m/s %7 yaw: %8 /s",
  "args0": [
    {
      "type": "input_dummy"
    },
    {
      "type": "field_number",
      "name": "velocity_x_m",
      "value": 0
    },
    {
      "type": "input_dummy"
    },
    {
      "type": "field_number",
      "name": "velocity_y_m",
      "value": 0
    },
    {
      "type": "input_dummy"
    },
    {
      "type": "field_number",
      "name": "velocity_z_m",
      "value": 0
    },
    {
      "type": "input_dummy"
    },
    {
      "type": "field_angle",
      "name": "rate_yaw",
      "angle": 90
    }
  ],
  "colour": 90,
  "tooltip": "",
  "helpUrl": ""
},
{
  "type": "stop",
  "message0": "stop and hover",
  "previousStatement": null,
  "nextStatement": null,
  "colour": 0,
  "tooltip": "",
  "helpUrl": ""
},
{
  "type": "start_circle",
  "message0": "start cricling %1 meter radius %2",
  "args0": [
    {
      "type": "field_number",
      "name": "radius",
      "value": 0
    },
    {
      "type": "field_dropdown",
      "name": "direction",
      "options": [
        [
          "right",
          "circle_right"
        ],
        [
          "left",
          "circle_left"
        ]
      ]
    }
  ],
  "previousStatement": null,
  "nextStatement": null,
  "colour": 90,
  "tooltip": "",
  "helpUrl": ""
},
{
  "type": "start_move",
  "message0": "start moving %1 at %2 m/s",
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
  "type": "start_turn",
  "message0": "start turning %1 at %2 degrees/s",
  "args0": [
    {
      "type": "field_dropdown",
      "name": "direction",
      "options": [
        [
          "right",
          "right"
        ],
        [
          "left",
          "left"
        ]
      ]
    },
    {
      "type": "field_angle",
      "name": "rate",
      "angle": 0
    }
  ],
  "previousStatement": null,
  "nextStatement": null,
  "colour": 90,
  "tooltip": "",
  "helpUrl": ""
},
{
  "type": "turn",
  "message0": "turn %1 %2 at %3 degrees/s",
  "args0": [
    {
      "type": "field_dropdown",
      "name": "direction",
      "options": [
        [
          "right",
          "\n\t\t\t\t\t\t\t\t\tright\n\t\t\t\t\t\t\t\t"
        ],
        [
          "left",
          "left"
        ]
      ]
    },
    {
      "type": "field_angle",
      "name": "degrees",
      "angle": 0
    },
    {
      "type": "field_angle",
      "name": "rate",
      "angle": 0
    }
  ],
  "previousStatement": null,
  "nextStatement": null,
  "colour": 90,
  "tooltip": "",
  "helpUrl": ""
},
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
}]