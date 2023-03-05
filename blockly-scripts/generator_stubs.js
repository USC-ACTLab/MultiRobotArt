Blockly.JavaScript['circle'] = function(block) {
  var number_radius_m = block.getFieldValue('radius_m');
  var number_velocity = block.getFieldValue('velocity');
  var angle_angle_degrees = block.getFieldValue('angle_degrees');
  var dropdown_direction = block.getFieldValue('direction');
  // TODO: Assemble JavaScript into code variable.
  var code = '...;\n';
  return code;
};

Blockly.JavaScript['start_linear_motion'] = function(block) {
  var number_velocity_x_m = block.getFieldValue('velocity_x_m');
  var number_velocity_y_m = block.getFieldValue('velocity_y_m');
  var number_velocity_z_m = block.getFieldValue('velocity_z_m');
  var angle_rate_yaw = block.getFieldValue('rate_yaw');
  // TODO: Assemble JavaScript into code variable.
  var code = '...;\n';
  return code;
};

Blockly.JavaScript['stop'] = function(block) {
  // TODO: Assemble JavaScript into code variable.
  var code = '...;\n';
  return code;
};

Blockly.JavaScript['start_circle'] = function(block) {
  var number_radius = block.getFieldValue('radius');
  var dropdown_direction = block.getFieldValue('direction');
  // TODO: Assemble JavaScript into code variable.
  var code = '...;\n';
  return code;
};

Blockly.JavaScript['start_move'] = function(block) {
  var dropdown_direction = block.getFieldValue('direction');
  var number_speed = block.getFieldValue('speed');
  // TODO: Assemble JavaScript into code variable.
  var code = '...;\n';
  return code;
};

Blockly.JavaScript['start_turn'] = function(block) {
  var dropdown_direction = block.getFieldValue('direction');
  var angle_rate = block.getFieldValue('rate');
  // TODO: Assemble JavaScript into code variable.
  var code = '...;\n';
  return code;
};

Blockly.JavaScript['turn'] = function(block) {
  var dropdown_direction = block.getFieldValue('direction');
  var angle_degrees = block.getFieldValue('degrees');
  var angle_rate = block.getFieldValue('rate');
  // TODO: Assemble JavaScript into code variable.
  var code = '...;\n';
  return code;
};

Blockly.JavaScript['set_default_height'] = function(block) {
  var number_default_height = block.getFieldValue('default_height');
  // TODO: Assemble JavaScript into code variable.
  var code = '...;\n';
  return code;
};

Blockly.JavaScript['go_to_xyz'] = function(block) {
  var value_x = Blockly.JavaScript.valueToCode(block, 'x', Blockly.JavaScript.ORDER_ATOMIC);
  var value_y = Blockly.JavaScript.valueToCode(block, 'y', Blockly.JavaScript.ORDER_ATOMIC);
  var value_z = Blockly.JavaScript.valueToCode(block, 'z', Blockly.JavaScript.ORDER_ATOMIC);
  var value_speed = Blockly.JavaScript.valueToCode(block, 'speed', Blockly.JavaScript.ORDER_ATOMIC);
  // TODO: Assemble JavaScript into code variable.
  var code = '...;\n';
  return code;
};

Blockly.JavaScript['go_to'] = function(block) {
  var number_x_pos = block.getFieldValue('x_pos');
  var number_y_pos = block.getFieldValue('y_pos');
  var number_z_pos = block.getFieldValue('z_pos');
  var number_speed = block.getFieldValue('speed');
  // TODO: Assemble JavaScript into code variable.
  var code = '...;\n';
  return code;
};

Blockly.JavaScript['get_a_position'] = function(block) {
  var dropdown_which = block.getFieldValue('which');
  var dropdown_name = block.getFieldValue('NAME');
  // TODO: Assemble JavaScript into code variable.
  var code = '...';
  // TODO: Change ORDER_NONE to the correct strength.
  return [code, Blockly.JavaScript.ORDER_NONE];
};

Blockly.JavaScript['move'] = function(block) {
  var dropdown_direction = block.getFieldValue('direction');
  var number_distance = block.getFieldValue('distance');
  var number_name = block.getFieldValue('NAME');
  // TODO: Assemble JavaScript into code variable.
  var code = '...;\n';
  return code;
};

Blockly.JavaScript['move_xyz'] = function(block) {
  var number_x = block.getFieldValue('x');
  var number_y = block.getFieldValue('y');
  var number_z = block.getFieldValue('z');
  var number_speed = block.getFieldValue('speed');
  // TODO: Assemble JavaScript into code variable.
  var code = '...;\n';
  return code;
};

Blockly.JavaScript['move_by_angles'] = function(block) {
  var angle_horizontal_angle = block.getFieldValue('horizontal angle');
  var angle_vertical_angle = block.getFieldValue('vertical angle');
  var number_distance = block.getFieldValue('distance');
  var number_name = block.getFieldValue('NAME');
  // TODO: Assemble JavaScript into code variable.
  var code = '...;\n';
  return code;
};

Blockly.JavaScript['set_default_speed'] = function(block) {
  var number_speed = block.getFieldValue('speed');
  // TODO: Assemble JavaScript into code variable.
  var code = '...;\n';
  return code;
};

Blockly.JavaScript['set_default_xy'] = function(block) {
  var dropdown_xy = block.getFieldValue('xy');
  var number_name = block.getFieldValue('NAME');
  // TODO: Assemble JavaScript into code variable.
  var code = '...;\n';
  return code;
};

Blockly.JavaScript['takeoff'] = function(block) {
  var number_height = block.getFieldValue('height');
  var number_speed = block.getFieldValue('speed');
  // TODO: Assemble JavaScript into code variable.
  var code = '...;\n';
  return code;
};

Blockly.JavaScript['land'] = function(block) {
  var number_height = block.getFieldValue('height');
  var number_speed = block.getFieldValue('speed');
  // TODO: Assemble JavaScript into code variable.
  var code = '...;\n';
  return code;
};

Blockly.JavaScript['get_position'] = function(block) {
  // TODO: Assemble JavaScript into code variable.
  var code = '...';
  // TODO: Change ORDER_NONE to the correct strength.
  return [code, Blockly.JavaScript.ORDER_NONE];
};