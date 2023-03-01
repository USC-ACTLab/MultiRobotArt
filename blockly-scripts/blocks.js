Blockly.Blocks['set_default_height'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("set default height to")
        .appendField(new Blockly.FieldNumber(0, 0), "default_height")
        .appendField("meters");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
 this.setTooltip("");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['go_to_xyz'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("go to");
    this.appendValueInput("x")
        .setCheck("Number")
        .appendField("----------------X:");
    this.appendValueInput("y")
        .setCheck("Number")
        .appendField("----------------Y:");
    this.appendValueInput("z")
        .setCheck("Number")
        .appendField("----------------Z:");
    this.appendValueInput("speed")
        .setCheck("Number")
        .appendField("at speed (m/s)");
    this.setInputsInline(false);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(90);
 this.setTooltip("");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['go_to'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("go to X:")
        .appendField(new Blockly.FieldNumber(0), "x_pos")
        .appendField("Y:")
        .appendField(new Blockly.FieldNumber(0), "y_pos")
        .appendField("Z:")
        .appendField(new Blockly.FieldNumber(0), "z_pos")
        .appendField("at")
        .appendField(new Blockly.FieldNumber(0), "speed")
        .appendField("m/s");
    this.setInputsInline(false);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(90);
 this.setTooltip("");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['get_a_position'] = {
  init: function() {
    this.appendDummyInput()
        .appendField(new Blockly.FieldDropdown([["default","default"], ["current","current"]]), "which")
        .appendField(new Blockly.FieldDropdown([["x","x position"], ["y","y position"], ["z","z position"]]), "NAME")
        .appendField("position");
    this.setOutput(true, null);
    this.setColour(230);
 this.setTooltip("");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['move'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("move")
        .appendField(new Blockly.FieldDropdown([["up","up"], ["down","down"], ["left","left"], ["right","right"], ["forward","forward"], ["backward","backward"]]), "direction")
        .appendField(new Blockly.FieldNumber(0), "distance")
        .appendField("meters at")
        .appendField(new Blockly.FieldNumber(0), "NAME")
        .appendField("m/s");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(90);
 this.setTooltip("");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['move_xyz'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("move X")
        .appendField(new Blockly.FieldNumber(0), "x")
        .appendField("meters");
    this.appendDummyInput()
        .appendField("move Y")
        .appendField(new Blockly.FieldNumber(0), "y")
        .appendField("meters");
    this.appendDummyInput()
        .appendField("move Z")
        .appendField(new Blockly.FieldNumber(0), "z")
        .appendField("meters");
    this.appendDummyInput()
        .appendField("at")
        .appendField(new Blockly.FieldNumber(0), "speed")
        .appendField("m/s");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(90);
 this.setTooltip("");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['move_by_angles'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("move towards");
    this.appendDummyInput()
        .appendField("horizontal angle")
        .appendField(new Blockly.FieldAngle(90), "horizontal angle");
    this.appendDummyInput()
        .appendField("vertical angle")
        .appendField(new Blockly.FieldAngle(90), "vertical angle");
    this.appendDummyInput()
        .appendField("distance")
        .appendField(new Blockly.FieldNumber(0), "distance");
    this.appendDummyInput()
        .appendField("at")
        .appendField(new Blockly.FieldNumber(0), "NAME")
        .appendField("m/s");
    this.setColour(90);
 this.setTooltip("");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['set_default_speed'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("set default speed to")
        .appendField(new Blockly.FieldNumber(0), "speed");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
 this.setTooltip("");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['set_default_xy'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("set default")
        .appendField(new Blockly.FieldDropdown([["x","x"], ["y","y"]]), "xy")
        .appendField("to")
        .appendField(new Blockly.FieldNumber(0), "NAME");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
 this.setTooltip("");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['takeoff'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("takeoff to")
        .appendField(new Blockly.FieldNumber(0), "height")
        .appendField("meters at")
        .appendField(new Blockly.FieldNumber(0), "speed")
        .appendField("m/s");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
 this.setTooltip("");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['land'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("land on")
        .appendField(new Blockly.FieldNumber(0), "height")
        .appendField("meter high object")
        .appendField("at")
        .appendField(new Blockly.FieldNumber(0), "speed")
        .appendField("m/s");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
 this.setTooltip("");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['get_position'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("get position");
    this.setOutput(true, null);
    this.setColour(230);
 this.setTooltip("");
 this.setHelpUrl("");
  }
};