[{
  "type": "____circle___",
  "message0": "%1 %2 \n\t\t\t\t\t\t\t\t\t\t\tm\n\t\t\t\t\t\t\t\t\t\t %3 \n\t\t\t\t\t\t\t\t\tat speed\n\t\t\t\t\t\t\t\t %4 \n\t\t\t\t\t\t\t\t\t\t\t\t\tm/s\n\t\t\t\t\t\t\t\t\t\t\t\t %5 \n\t\t\t\t\t\t\t\t\t\t\tfor\n\t\t\t\t\t\t\t\t\t\t %6 \n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\tdegrees,\n\t\t\t\t\t\t\t\t\t\t\t\t\t\t %7 %8",
  "args0": [
    {
      "type": "field_label_serializable",
      "name": "\n\t\t\t\t\t\t\tCircular motion\n\t\t\t\t\t\t",
      "text": "\n\t\t\t\t\t\t\tGo in circle of radius\n\t\t\t\t\t\t"
    },
    {
      "type": "field_number",
      "name": "\n\t\t\t\t\t\t\t\t\tradius_m\n\t\t\t\t\t\t\t\t",
      "value": 0,
      "min": 0
    },
    {
      "type": "input_dummy"
    },
    {
      "type": "field_number",
      "name": "\n\t\t\t\t\t\t\t\t\t\t\tvelocity\n\t\t\t\t\t\t\t\t\t\t",
      "value": 0.2
    },
    {
      "type": "input_dummy"
    },
    {
      "type": "field_angle",
      "name": "\n\t\t\t\t\t\t\t\t\t\t\t\t\tangle_degrees\n\t\t\t\t\t\t\t\t\t\t\t\t",
      "angle": 90
    },
    {
      "type": "input_dummy"
    },
    {
      "type": "field_dropdown",
      "name": "\n\t\t\t\t\t\t\t\t\t\t\t\t\tdirection\n\t\t\t\t\t\t\t\t\t\t\t\t",
      "options": [
        [
          "\n\t\t\t\t\t\t\t\t\t\t\t\t\tcounterclockwise\n\t\t\t\t\t\t\t\t\t\t\t\t",
          "\n\t\t\t\t\t\t\t\t\t\t\t\t\tccw\n\t\t\t\t\t\t\t\t\t\t\t\t"
        ],
        [
          "\n\t\t\t\t\t\t\t\t\t\t\t\t\tclockwise\n\t\t\t\t\t\t\t\t\t\t\t\t",
          "\n\t\t\t\t\t\t\t\t\t\t\t\t\tcw\n\t\t\t\t\t\t\t\t\t\t\t\t"
        ]
      ]
    }
  ],
  "colour": 90,
  "tooltip": "\n\t\t\t\t",
  "helpUrl": "\n\t\t\t\t"
},
{
  "type": "____start_linear_motion___",
  "message0": "\n\t\t\t\t\t\t\tStart linear motion with speeds in\n\t\t\t\t\t\t %1 \n\t\t\t\t\t\t\t\t\tx direction:\n\t\t\t\t\t\t\t\t %2 \n\t\t\t\t\t\t\t\t\t\t\t\t\tm/s\n\t\t\t\t\t\t\t\t\t\t\t\t %3 \n\t\t\t\t\t\t\t\t\t\t\ty direction:\n\t\t\t\t\t\t\t\t\t\t %4 \n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\tm/s\n\t\t\t\t\t\t\t\t\t\t\t\t\t\t %5 \n\t\t\t\t\t\t\t\t\t\t\t\t\tz direction:\n\t\t\t\t\t\t\t\t\t\t\t\t %6 \n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\tm/s\n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t %7 \n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\tyaw:\n\t\t\t\t\t\t\t\t\t\t\t\t\t\t %8 \n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t/s\n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t",
  "args0": [
    {
      "type": "input_dummy"
    },
    {
      "type": "field_number",
      "name": "\n\t\t\t\t\t\t\t\t\t\t\tvelocity_x_m\n\t\t\t\t\t\t\t\t\t\t",
      "value": 0
    },
    {
      "type": "input_dummy"
    },
    {
      "type": "field_number",
      "name": "\n\t\t\t\t\t\t\t\t\t\t\t\t\tvelocity_y_m\n\t\t\t\t\t\t\t\t\t\t\t\t",
      "value": 0
    },
    {
      "type": "input_dummy"
    },
    {
      "type": "field_number",
      "name": "\n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\tvelocity_z_m\n\t\t\t\t\t\t\t\t\t\t\t\t\t\t",
      "value": 0
    },
    {
      "type": "input_dummy"
    },
    {
      "type": "field_angle",
      "name": "\n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\trate_yaw\n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t",
      "angle": 90
    }
  ],
  "colour": 90,
  "tooltip": "\n\t\t\t\t",
  "helpUrl": "\n\t\t\t\t"
},
{
  "type": "____stop___",
  "message0": "\n\t\t\t\t\t\t\tstop and hover\n\t\t\t\t\t\t",
  "colour": 0,
  "tooltip": "\n\t\t\t\t",
  "helpUrl": "\n\t\t\t\t"
},
{
  "type": "____start_circle___",
  "message0": "\n\t\t\t\t\t\t\tstart cricling\n\t\t\t\t\t\t %1 \n\t\t\t\t\t\t\t\t\t\t\tmeter radius\n\t\t\t\t\t\t\t\t\t\t %2",
  "args0": [
    {
      "type": "field_number",
      "name": "\n\t\t\t\t\t\t\t\t\tradius\n\t\t\t\t\t\t\t\t",
      "value": 0
    },
    {
      "type": "field_dropdown",
      "name": "\n\t\t\t\t\t\t\t\t\t\t\t\t\tdirection\n\t\t\t\t\t\t\t\t\t\t\t\t",
      "options": [
        [
          "\n\t\t\t\t\t\t\t\t\t\t\t\t\tright\n\t\t\t\t\t\t\t\t\t\t\t\t",
          "\n\t\t\t\t\t\t\t\t\t\t\t\t\tcircle_right\n\t\t\t\t\t\t\t\t\t\t\t\t"
        ],
        [
          "\n\t\t\t\t\t\t\t\t\t\t\t\t\tleft\n\t\t\t\t\t\t\t\t\t\t\t\t",
          "\n\t\t\t\t\t\t\t\t\t\t\t\t\tcircle_left\n\t\t\t\t\t\t\t\t\t\t\t\t"
        ]
      ]
    }
  ],
  "colour": 90,
  "tooltip": "\n\t\t\t\t",
  "helpUrl": "\n\t\t\t\t"
},
{
  "type": "____start_move___",
  "message0": "\n\t\t\t\t\t\t\tstart moving\n\t\t\t\t\t\t %1 \n\t\t\t\t\t\t\t\t\t\t\tat\n\t\t\t\t\t\t\t\t\t\t %2 \n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\tm/s\n\t\t\t\t\t\t\t\t\t\t\t\t\t\t",
  "args0": [
    {
      "type": "field_dropdown",
      "name": "\n\t\t\t\t\t\t\t\t\tdirection\n\t\t\t\t\t\t\t\t",
      "options": [
        [
          "\n\t\t\t\t\t\t\t\t\tup\n\t\t\t\t\t\t\t\t",
          "\n\t\t\t\t\t\t\t\t\tup\n\t\t\t\t\t\t\t\t"
        ],
        [
          "\n\t\t\t\t\t\t\t\t\tdown\n\t\t\t\t\t\t\t\t",
          "\n\t\t\t\t\t\t\t\t\tdown\n\t\t\t\t\t\t\t\t"
        ],
        [
          "\n\t\t\t\t\t\t\t\t\tleft\n\t\t\t\t\t\t\t\t",
          "\n\t\t\t\t\t\t\t\t\tleft\n\t\t\t\t\t\t\t\t"
        ],
        [
          "\n\t\t\t\t\t\t\t\t\tright\n\t\t\t\t\t\t\t\t",
          "\n\t\t\t\t\t\t\t\t\tright\n\t\t\t\t\t\t\t\t"
        ],
        [
          "\n\t\t\t\t\t\t\t\t\tforward\n\t\t\t\t\t\t\t\t",
          "\n\t\t\t\t\t\t\t\t\tforward\n\t\t\t\t\t\t\t\t"
        ],
        [
          "\n\t\t\t\t\t\t\t\t\tbackward\n\t\t\t\t\t\t\t\t",
          "\n\t\t\t\t\t\t\t\t\tbackward\n\t\t\t\t\t\t\t\t"
        ]
      ]
    },
    {
      "type": "field_number",
      "name": "\n\t\t\t\t\t\t\t\t\t\t\t\t\tspeed\n\t\t\t\t\t\t\t\t\t\t\t\t",
      "value": 0
    }
  ],
  "colour": 90,
  "tooltip": "\n\t\t\t\t",
  "helpUrl": "\n\t\t\t\t"
},
{
  "type": "____start_turn___",
  "message0": "\n\t\t\t\t\t\t\tstart turning\n\t\t\t\t\t\t %1 \n\t\t\t\t\t\t\t\t\t\t\tat\n\t\t\t\t\t\t\t\t\t\t %2 \n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\tdegrees/s\n\t\t\t\t\t\t\t\t\t\t\t\t\t\t",
  "args0": [
    {
      "type": "field_dropdown",
      "name": "\n\t\t\t\t\t\t\t\t\tdirection\n\t\t\t\t\t\t\t\t",
      "options": [
        [
          "\n\t\t\t\t\t\t\t\t\tright\n\t\t\t\t\t\t\t\t",
          "\n\t\t\t\t\t\t\t\t\tright\n\t\t\t\t\t\t\t\t"
        ],
        [
          "\n\t\t\t\t\t\t\t\t\tleft\n\t\t\t\t\t\t\t\t",
          "\n\t\t\t\t\t\t\t\t\tleft\n\t\t\t\t\t\t\t\t"
        ]
      ]
    },
    {
      "type": "field_angle",
      "name": "\n\t\t\t\t\t\t\t\t\t\t\t\t\trate\n\t\t\t\t\t\t\t\t\t\t\t\t",
      "angle": 0
    }
  ],
  "colour": 90,
  "tooltip": "\n\t\t\t\t",
  "helpUrl": "\n\t\t\t\t"
},
{
  "type": "____turn___",
  "message0": "\n\t\t\t\t\t\t\tturn\n\t\t\t\t\t\t %1 %2 \n\t\t\t\t\t\t\t\t\t\t\t\t\tat\n\t\t\t\t\t\t\t\t\t\t\t\t %3 \n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\tdegrees/s\n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t",
  "args0": [
    {
      "type": "field_dropdown",
      "name": "\n\t\t\t\t\t\t\t\t\tdirection\n\t\t\t\t\t\t\t\t",
      "options": [
        [
          "\n\t\t\t\t\t\t\t\t\tright\n\t\t\t\t\t\t\t\t",
          "\n\t\t\t\t\t\t\t\t\tright\n\t\t\t\t\t\t\t\t"
        ],
        [
          "\n\t\t\t\t\t\t\t\t\tleft\n\t\t\t\t\t\t\t\t",
          "\n\t\t\t\t\t\t\t\t\tleft\n\t\t\t\t\t\t\t\t"
        ]
      ]
    },
    {
      "type": "field_angle",
      "name": "\n\t\t\t\t\t\t\t\t\t\t\tdegrees\n\t\t\t\t\t\t\t\t\t\t",
      "angle": 0
    },
    {
      "type": "field_angle",
      "name": "\n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\trate\n\t\t\t\t\t\t\t\t\t\t\t\t\t\t",
      "angle": 0
    }
  ],
  "colour": 90,
  "tooltip": "\n\t\t\t\t",
  "helpUrl": "\n\t\t\t\t"
},
{
  "type": "____set_default_height___",
  "message0": "\n\t\t\t\t\t\t\tset default height to\n\t\t\t\t\t\t %1 \n\t\t\t\t\t\t\t\t\t\t\tmeters\n\t\t\t\t\t\t\t\t\t\t",
  "args0": [
    {
      "type": "field_number",
      "name": "\n\t\t\t\t\t\t\t\t\tdefault_height\n\t\t\t\t\t\t\t\t",
      "value": 0,
      "min": 0
    }
  ],
  "colour": 230,
  "tooltip": "\n\t\t\t\t",
  "helpUrl": "\n\t\t\t\t"
},
{
  "type": "____go_to_xyz___",
  "message0": "\n\t\t\t\t\t\t\tgo to\n\t\t\t\t\t\t %1 \n\t\t\t\t\t\t\t\t\t----------------X:\n\t\t\t\t\t\t\t\t %2 \n\t\t\t\t\t\t\t\t\t\t\t----------------Y:\n\t\t\t\t\t\t\t\t\t\t %3 \n\t\t\t\t\t\t\t\t\t\t\t\t\t----------------Z:\n\t\t\t\t\t\t\t\t\t\t\t\t %4 \n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\tat speed (m/s)\n\t\t\t\t\t\t\t\t\t\t\t\t\t\t %5",
  "args0": [
    {
      "type": "input_dummy"
    },
    {
      "type": "input_value",
      "name": "\n\t\t\t\t\t\t\tx\n\t\t\t\t\t\t",
      "check": "Number"
    },
    {
      "type": "input_value",
      "name": "\n\t\t\t\t\t\t\t\t\ty\n\t\t\t\t\t\t\t\t",
      "check": "Number"
    },
    {
      "type": "input_value",
      "name": "\n\t\t\t\t\t\t\t\t\t\t\tz\n\t\t\t\t\t\t\t\t\t\t",
      "check": "Number"
    },
    {
      "type": "input_value",
      "name": "\n\t\t\t\t\t\t\t\t\t\t\t\t\tspeed\n\t\t\t\t\t\t\t\t\t\t\t\t",
      "check": "Number"
    }
  ],
  "colour": 90,
  "tooltip": "\n\t\t\t\t",
  "helpUrl": "\n\t\t\t\t"
},
{
  "type": "____go_to___",
  "message0": "\n\t\t\t\t\t\t\tgo to X:\n\t\t\t\t\t\t %1 \n\t\t\t\t\t\t\t\t\t\t\tY:\n\t\t\t\t\t\t\t\t\t\t %2 \n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\tZ:\n\t\t\t\t\t\t\t\t\t\t\t\t\t\t %3 \n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\tat\n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t %4 \n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\tm/s\n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t",
  "args0": [
    {
      "type": "field_number",
      "name": "\n\t\t\t\t\t\t\t\t\tx_pos\n\t\t\t\t\t\t\t\t",
      "value": 0
    },
    {
      "type": "field_number",
      "name": "\n\t\t\t\t\t\t\t\t\t\t\t\t\ty_pos\n\t\t\t\t\t\t\t\t\t\t\t\t",
      "value": 0
    },
    {
      "type": "field_number",
      "name": "\n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\tz_pos\n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t",
      "value": 0
    },
    {
      "type": "field_number",
      "name": "\n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\tspeed\n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t",
      "value": 0
    }
  ],
  "colour": 90,
  "tooltip": "\n\t\t\t\t",
  "helpUrl": "\n\t\t\t\t"
},
{
  "type": "____get_a_position___",
  "message0": "%1 %2 \n\t\t\t\t\t\t\t\t\t\t\tposition\n\t\t\t\t\t\t\t\t\t\t",
  "args0": [
    {
      "type": "field_dropdown",
      "name": "\n\t\t\t\t\t\t\twhich\n\t\t\t\t\t\t",
      "options": [
        [
          "\n\t\t\t\t\t\t\tdefault\n\t\t\t\t\t\t",
          "\n\t\t\t\t\t\t\tdefault\n\t\t\t\t\t\t"
        ],
        [
          "\n\t\t\t\t\t\t\tcurrent\n\t\t\t\t\t\t",
          "\n\t\t\t\t\t\t\tcurrent\n\t\t\t\t\t\t"
        ]
      ]
    },
    {
      "type": "field_dropdown",
      "name": "\n\t\t\t\t\t\t\t\t\tNAME\n\t\t\t\t\t\t\t\t",
      "options": [
        [
          "\n\t\t\t\t\t\t\t\t\tx\n\t\t\t\t\t\t\t\t",
          "\n\t\t\t\t\t\t\t\t\tx position\n\t\t\t\t\t\t\t\t"
        ],
        [
          "\n\t\t\t\t\t\t\t\t\ty\n\t\t\t\t\t\t\t\t",
          "\n\t\t\t\t\t\t\t\t\ty position\n\t\t\t\t\t\t\t\t"
        ],
        [
          "\n\t\t\t\t\t\t\t\t\tz\n\t\t\t\t\t\t\t\t",
          "\n\t\t\t\t\t\t\t\t\tz position\n\t\t\t\t\t\t\t\t"
        ]
      ]
    }
  ],
  "colour": 230,
  "tooltip": "\n\t\t\t\t",
  "helpUrl": "\n\t\t\t\t"
},
{
  "type": "____move___",
  "message0": "\n\t\t\t\t\t\t\tmove\n\t\t\t\t\t\t %1 %2 \n\t\t\t\t\t\t\t\t\t\t\t\t\tmeters at\n\t\t\t\t\t\t\t\t\t\t\t\t %3 \n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\tm/s\n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t",
  "args0": [
    {
      "type": "field_dropdown",
      "name": "\n\t\t\t\t\t\t\t\t\tdirection\n\t\t\t\t\t\t\t\t",
      "options": [
        [
          "\n\t\t\t\t\t\t\t\t\tup\n\t\t\t\t\t\t\t\t",
          "\n\t\t\t\t\t\t\t\t\tup\n\t\t\t\t\t\t\t\t"
        ],
        [
          "\n\t\t\t\t\t\t\t\t\tdown\n\t\t\t\t\t\t\t\t",
          "\n\t\t\t\t\t\t\t\t\tdown\n\t\t\t\t\t\t\t\t"
        ],
        [
          "\n\t\t\t\t\t\t\t\t\tleft\n\t\t\t\t\t\t\t\t",
          "\n\t\t\t\t\t\t\t\t\tleft\n\t\t\t\t\t\t\t\t"
        ],
        [
          "\n\t\t\t\t\t\t\t\t\tright\n\t\t\t\t\t\t\t\t",
          "\n\t\t\t\t\t\t\t\t\tright\n\t\t\t\t\t\t\t\t"
        ],
        [
          "\n\t\t\t\t\t\t\t\t\tforward\n\t\t\t\t\t\t\t\t",
          "\n\t\t\t\t\t\t\t\t\tforward\n\t\t\t\t\t\t\t\t"
        ],
        [
          "\n\t\t\t\t\t\t\t\t\tbackward\n\t\t\t\t\t\t\t\t",
          "\n\t\t\t\t\t\t\t\t\tbackward\n\t\t\t\t\t\t\t\t"
        ]
      ]
    },
    {
      "type": "field_number",
      "name": "\n\t\t\t\t\t\t\t\t\t\t\tdistance\n\t\t\t\t\t\t\t\t\t\t",
      "value": 0
    },
    {
      "type": "field_number",
      "name": "\n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\tNAME\n\t\t\t\t\t\t\t\t\t\t\t\t\t\t",
      "value": 0
    }
  ],
  "colour": 90,
  "tooltip": "\n\t\t\t\t",
  "helpUrl": "\n\t\t\t\t"
},
{
  "type": "____move_xyz___",
  "message0": "\n\t\t\t\t\t\t\tmove X\n\t\t\t\t\t\t %1 \n\t\t\t\t\t\t\t\t\t\t\tmeters\n\t\t\t\t\t\t\t\t\t\t %2 \n\t\t\t\t\t\t\t\t\tmove Y\n\t\t\t\t\t\t\t\t %3 \n\t\t\t\t\t\t\t\t\t\t\t\t\tmeters\n\t\t\t\t\t\t\t\t\t\t\t\t %4 \n\t\t\t\t\t\t\t\t\t\t\tmove Z\n\t\t\t\t\t\t\t\t\t\t %5 \n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\tmeters\n\t\t\t\t\t\t\t\t\t\t\t\t\t\t %6 \n\t\t\t\t\t\t\t\t\t\t\t\t\tat\n\t\t\t\t\t\t\t\t\t\t\t\t %7 \n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\tm/s\n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t",
  "args0": [
    {
      "type": "field_number",
      "name": "\n\t\t\t\t\t\t\t\t\tx\n\t\t\t\t\t\t\t\t",
      "value": 0
    },
    {
      "type": "input_dummy"
    },
    {
      "type": "field_number",
      "name": "\n\t\t\t\t\t\t\t\t\t\t\ty\n\t\t\t\t\t\t\t\t\t\t",
      "value": 0
    },
    {
      "type": "input_dummy"
    },
    {
      "type": "field_number",
      "name": "\n\t\t\t\t\t\t\t\t\t\t\t\t\tz\n\t\t\t\t\t\t\t\t\t\t\t\t",
      "value": 0
    },
    {
      "type": "input_dummy"
    },
    {
      "type": "field_number",
      "name": "\n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\tspeed\n\t\t\t\t\t\t\t\t\t\t\t\t\t\t",
      "value": 0
    }
  ],
  "colour": 90,
  "tooltip": "\n\t\t\t\t",
  "helpUrl": "\n\t\t\t\t"
},
{
  "type": "____move_by_angles___",
  "message0": "\n\t\t\t\t\t\t\tmove towards\n\t\t\t\t\t\t %1 \n\t\t\t\t\t\t\t\t\thorizontal angle\n\t\t\t\t\t\t\t\t %2 %3 \n\t\t\t\t\t\t\t\t\t\t\tvertical angle\n\t\t\t\t\t\t\t\t\t\t %4 %5 \n\t\t\t\t\t\t\t\t\t\t\t\t\tdistance\n\t\t\t\t\t\t\t\t\t\t\t\t %6 %7 \n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\tat\n\t\t\t\t\t\t\t\t\t\t\t\t\t\t %8 \n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\tm/s\n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t",
  "args0": [
    {
      "type": "input_dummy"
    },
    {
      "type": "field_angle",
      "name": "\n\t\t\t\t\t\t\t\t\t\t\thorizontal angle\n\t\t\t\t\t\t\t\t\t\t",
      "angle": 90
    },
    {
      "type": "input_dummy"
    },
    {
      "type": "field_angle",
      "name": "\n\t\t\t\t\t\t\t\t\t\t\t\t\tvertical angle\n\t\t\t\t\t\t\t\t\t\t\t\t",
      "angle": 90
    },
    {
      "type": "input_dummy"
    },
    {
      "type": "field_number",
      "name": "\n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\tdistance\n\t\t\t\t\t\t\t\t\t\t\t\t\t\t",
      "value": 0
    },
    {
      "type": "input_dummy"
    },
    {
      "type": "field_number",
      "name": "\n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\tNAME\n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t",
      "value": 0
    }
  ],
  "colour": 90,
  "tooltip": "\n\t\t\t\t",
  "helpUrl": "\n\t\t\t\t"
},
{
  "type": "____set_default_speed___",
  "message0": "\n\t\t\t\t\t\t\tset default speed to\n\t\t\t\t\t\t %1",
  "args0": [
    {
      "type": "field_number",
      "name": "\n\t\t\t\t\t\t\t\t\tspeed\n\t\t\t\t\t\t\t\t",
      "value": 0
    }
  ],
  "colour": 230,
  "tooltip": "\n\t\t\t\t",
  "helpUrl": "\n\t\t\t\t"
},
{
  "type": "____set_default_xy___",
  "message0": "\n\t\t\t\t\t\t\tset default\n\t\t\t\t\t\t %1 \n\t\t\t\t\t\t\t\t\t\t\tto\n\t\t\t\t\t\t\t\t\t\t %2",
  "args0": [
    {
      "type": "field_dropdown",
      "name": "\n\t\t\t\t\t\t\t\t\txy\n\t\t\t\t\t\t\t\t",
      "options": [
        [
          "\n\t\t\t\t\t\t\t\t\tx\n\t\t\t\t\t\t\t\t",
          "\n\t\t\t\t\t\t\t\t\tx\n\t\t\t\t\t\t\t\t"
        ],
        [
          "\n\t\t\t\t\t\t\t\t\ty\n\t\t\t\t\t\t\t\t",
          "\n\t\t\t\t\t\t\t\t\ty\n\t\t\t\t\t\t\t\t"
        ]
      ]
    },
    {
      "type": "field_number",
      "name": "\n\t\t\t\t\t\t\t\t\t\t\t\t\tNAME\n\t\t\t\t\t\t\t\t\t\t\t\t",
      "value": 0
    }
  ],
  "colour": 230,
  "tooltip": "\n\t\t\t\t",
  "helpUrl": "\n\t\t\t\t"
},
{
  "type": "____takeoff___",
  "message0": "\n\t\t\t\t\t\t\ttakeoff to\n\t\t\t\t\t\t %1 \n\t\t\t\t\t\t\t\t\t\t\tmeters at\n\t\t\t\t\t\t\t\t\t\t %2 \n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\tm/s\n\t\t\t\t\t\t\t\t\t\t\t\t\t\t",
  "args0": [
    {
      "type": "field_number",
      "name": "\n\t\t\t\t\t\t\t\t\theight\n\t\t\t\t\t\t\t\t",
      "value": 0
    },
    {
      "type": "field_number",
      "name": "\n\t\t\t\t\t\t\t\t\t\t\t\t\tspeed\n\t\t\t\t\t\t\t\t\t\t\t\t",
      "value": 0
    }
  ],
  "colour": 230,
  "tooltip": "\n\t\t\t\t",
  "helpUrl": "\n\t\t\t\t"
},
{
  "type": "____land___",
  "message0": "\n\t\t\t\t\t\t\tland on\n\t\t\t\t\t\t %1 \n\t\t\t\t\t\t\t\t\t\t\tmeter high object\n\t\t\t\t\t\t\t\t\t\t \n\t\t\t\t\t\t\t\t\t\t\t\t\tat\n\t\t\t\t\t\t\t\t\t\t\t\t %2 \n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\tm/s\n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t",
  "args0": [
    {
      "type": "field_number",
      "name": "\n\t\t\t\t\t\t\t\t\theight\n\t\t\t\t\t\t\t\t",
      "value": 0
    },
    {
      "type": "field_number",
      "name": "\n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\tspeed\n\t\t\t\t\t\t\t\t\t\t\t\t\t\t",
      "value": 0
    }
  ],
  "colour": 230,
  "tooltip": "\n\t\t\t\t",
  "helpUrl": "\n\t\t\t\t"
},
{
  "type": "____get_position___",
  "message0": "\n\t\t\t\t\t\t\tget position\n\t\t\t\t\t\t",
  "colour": 230,
  "tooltip": "\n\t\t\t\t",
  "helpUrl": "\n\t\t\t\t"
},
{
  "type": "circle",
  "message0": "%1 %2 \n\t\t\t\t\t\t\t\t\t\t\tm\n\t\t\t\t\t\t\t\t\t\t\n                    \n                     %3 \n\t\t\t\t\t\t\t\t\tat speed\n\t\t\t\t\t\t\t\t\n                \n                 %4 \n\t\t\t\t\t\t\t\t\t\t\t\t\tm/s\n\t\t\t\t\t\t\t\t\t\t\t\t\n                        \n                         %5 \n\t\t\t\t\t\t\t\t\t\t\tfor\n\t\t\t\t\t\t\t\t\t\t\n                    \n                     %6 \n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\tdegrees,\n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\n                            \n                             %7 %8",
  "args0": [
    {
      "type": "field_label_serializable",
      "name": "\n\t\t\t\t\t\t\tCircular motion\n\t\t\t\t\t\t\n            \n            ",
      "text": "\n\t\t\t\t\t\t\tGo in circle of radius\n\t\t\t\t\t\t\n            \n            "
    },
    {
      "type": "field_number",
      "name": "\n\t\t\t\t\t\t\t\t\tradius_m\n\t\t\t\t\t\t\t\t\n                \n                ",
      "value": 0,
      "min": 0
    },
    {
      "type": "input_dummy"
    },
    {
      "type": "field_number",
      "name": "\n\t\t\t\t\t\t\t\t\t\t\tvelocity\n\t\t\t\t\t\t\t\t\t\t\n                    \n                    ",
      "value": 0.2
    },
    {
      "type": "input_dummy"
    },
    {
      "type": "field_angle",
      "name": "\n\t\t\t\t\t\t\t\t\t\t\t\t\tangle_degrees\n\t\t\t\t\t\t\t\t\t\t\t\t\n                        \n                        ",
      "angle": 90
    },
    {
      "type": "input_dummy"
    },
    {
      "type": "field_dropdown",
      "name": "\n\t\t\t\t\t\t\t\t\t\t\t\t\tdirection\n\t\t\t\t\t\t\t\t\t\t\t\t\n                        \n                        ",
      "options": [
        [
          "\n\t\t\t\t\t\t\t\t\t\t\t\t\tcounterclockwise\n\t\t\t\t\t\t\t\t\t\t\t\t\n                        \n                        ",
          "\n\t\t\t\t\t\t\t\t\t\t\t\t\tccw\n\t\t\t\t\t\t\t\t\t\t\t\t\n                        \n                        "
        ],
        [
          "\n\t\t\t\t\t\t\t\t\t\t\t\t\tclockwise\n\t\t\t\t\t\t\t\t\t\t\t\t\n                        \n                        ",
          "\n\t\t\t\t\t\t\t\t\t\t\t\t\tcw\n\t\t\t\t\t\t\t\t\t\t\t\t\n                        \n                        "
        ]
      ]
    }
  ],
  "tooltip": "\n\t\t\t\t\n        \n        ",
  "helpUrl": "\n\t\t\t\t\n        \n        "
}]