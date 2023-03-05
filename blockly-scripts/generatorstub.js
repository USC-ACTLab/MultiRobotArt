Blockly.JavaScript['____circle___'] = function(block) {
  var number___________radius_m_________ = block.getFieldValue('
									radius_m
								');
  var number_____________velocity___________ = block.getFieldValue('
											velocity
										');
  var angle_______________angle_degrees_____________ = block.getFieldValue('
													angle_degrees
												');
  var dropdown_______________direction_____________ = block.getFieldValue('
													direction
												');
  // TODO: Assemble JavaScript into code variable.
  var code = '...;\n';
  return code;
};

Blockly.JavaScript['____start_linear_motion___'] = function(block) {
  var number_____________velocity_x_m___________ = block.getFieldValue('
											velocity_x_m
										');
  var number_______________velocity_y_m_____________ = block.getFieldValue('
													velocity_y_m
												');
  var number_________________velocity_z_m_______________ = block.getFieldValue('
															velocity_z_m
														');
  var angle___________________rate_yaw_________________ = block.getFieldValue('
																	rate_yaw
																');
  // TODO: Assemble JavaScript into code variable.
  var code = '...;\n';
  return code;
};

Blockly.JavaScript['____stop___'] = function(block) {
  // TODO: Assemble JavaScript into code variable.
  var code = '...;\n';
  return code;
};

Blockly.JavaScript['____start_circle___'] = function(block) {
  var number___________radius_________ = block.getFieldValue('
									radius
								');
  var dropdown_______________direction_____________ = block.getFieldValue('
													direction
												');
  // TODO: Assemble JavaScript into code variable.
  var code = '...;\n';
  return code;
};

Blockly.JavaScript['____start_move___'] = function(block) {
  var dropdown___________direction_________ = block.getFieldValue('
									direction
								');
  var number_______________speed_____________ = block.getFieldValue('
													speed
												');
  // TODO: Assemble JavaScript into code variable.
  var code = '...;\n';
  return code;
};

Blockly.JavaScript['____start_turn___'] = function(block) {
  var dropdown___________direction_________ = block.getFieldValue('
									direction
								');
  var angle_______________rate_____________ = block.getFieldValue('
													rate
												');
  // TODO: Assemble JavaScript into code variable.
  var code = '...;\n';
  return code;
};

Blockly.JavaScript['____turn___'] = function(block) {
  var dropdown___________direction_________ = block.getFieldValue('
									direction
								');
  var angle_____________degrees___________ = block.getFieldValue('
											degrees
										');
  var angle_________________rate_______________ = block.getFieldValue('
															rate
														');
  // TODO: Assemble JavaScript into code variable.
  var code = '...;\n';
  return code;
};

Blockly.JavaScript['____set_default_height___'] = function(block) {
  var number___________default_height_________ = block.getFieldValue('
									default_height
								');
  // TODO: Assemble JavaScript into code variable.
  var code = '...;\n';
  return code;
};

Blockly.JavaScript['____go_to_xyz___'] = function(block) {
  var value_________x_______ = Blockly.JavaScript.valueToCode(block, '
							x
						', Blockly.JavaScript.ORDER_ATOMIC);
  var value___________y_________ = Blockly.JavaScript.valueToCode(block, '
									y
								', Blockly.JavaScript.ORDER_ATOMIC);
  var value_____________z___________ = Blockly.JavaScript.valueToCode(block, '
											z
										', Blockly.JavaScript.ORDER_ATOMIC);
  var value_______________speed_____________ = Blockly.JavaScript.valueToCode(block, '
													speed
												', Blockly.JavaScript.ORDER_ATOMIC);
  // TODO: Assemble JavaScript into code variable.
  var code = '...;\n';
  return code;
};

Blockly.JavaScript['____go_to___'] = function(block) {
  var number___________x_pos_________ = block.getFieldValue('
									x_pos
								');
  var number_______________y_pos_____________ = block.getFieldValue('
													y_pos
												');
  var number___________________z_pos_________________ = block.getFieldValue('
																	z_pos
																');
  var number_______________________speed_____________________ = block.getFieldValue('
																					speed
																				');
  // TODO: Assemble JavaScript into code variable.
  var code = '...;\n';
  return code;
};

Blockly.JavaScript['____get_a_position___'] = function(block) {
  var dropdown_________which_______ = block.getFieldValue('
							which
						');
  var dropdown___________name_________ = block.getFieldValue('
									NAME
								');
  // TODO: Assemble JavaScript into code variable.
  var code = '...;\n';
  return code;
};

Blockly.JavaScript['____move___'] = function(block) {
  var dropdown___________direction_________ = block.getFieldValue('
									direction
								');
  var number_____________distance___________ = block.getFieldValue('
											distance
										');
  var number_________________name_______________ = block.getFieldValue('
															NAME
														');
  // TODO: Assemble JavaScript into code variable.
  var code = "move(direction, distance, name)"
  return code;
};

Blockly.JavaScript['____move_xyz___'] = function(block) {
  var number___________x_________ = block.getFieldValue('
									x
								');
  var number_____________y___________ = block.getFieldValue('
											y
										');
  var number_______________z_____________ = block.getFieldValue('
													z
												');
  var number_________________speed_______________ = block.getFieldValue('
															speed
														');
  // TODO: Assemble JavaScript into code variable.
  var code = '...;\n';
  return code;
};

Blockly.JavaScript['____move_by_angles___'] = function(block) {
  var angle_____________horizontal_angle___________ = block.getFieldValue('
											horizontal angle
										');
  var angle_______________vertical_angle_____________ = block.getFieldValue('
													vertical angle
												');
  var number_________________distance_______________ = block.getFieldValue('
															distance
														');
  var number___________________name_________________ = block.getFieldValue('
																	NAME
																');
  // TODO: Assemble JavaScript into code variable.
  var code = '...;\n';
  return code;
};

Blockly.JavaScript['____set_default_speed___'] = function(block) {
  var number___________speed_________ = block.getFieldValue('
									speed
								');
  // TODO: Assemble JavaScript into code variable.
  var code = '...;\n';
  return code;
};

Blockly.JavaScript['____set_default_xy___'] = function(block) {
  var dropdown___________xy_________ = block.getFieldValue('
									xy
								');
  var number_______________name_____________ = block.getFieldValue('
													NAME
												');
  // TODO: Assemble JavaScript into code variable.
  var code = '...;\n';
  return code;
};

Blockly.JavaScript['____takeoff___'] = function(block) {
  var number___________height_________ = block.getFieldValue('
									height
								');
  var number_______________speed_____________ = block.getFieldValue('
													speed
												');
  // TODO: Assemble JavaScript into code variable.
  var code = '...;\n';
  return code;
};

Blockly.JavaScript['____land___'] = function(block) {
  var number___________height_________ = block.getFieldValue('
									height
								');
  var number_________________speed_______________ = block.getFieldValue('
															speed
														');
  // TODO: Assemble JavaScript into code variable.
  var code = '...;\n';
  return code;
};

Blockly.JavaScript['____get_position___'] = function(block) {
  // TODO: Assemble JavaScript into code variable.
  var code = '...;\n';
  return code;
};

Blockly.JavaScript['circle'] = function(block) {
  var number___________radius_m___________________________________________ = block.getFieldValue('
									radius_m
								
                
                ');
  var number_____________velocity_____________________________________________________ = block.getFieldValue('
											velocity
										
                    
                    ');
  var angle_______________angle_degrees_______________________________________________________________ = block.getFieldValue('
													angle_degrees
												
                        
                        ');
  var dropdown_______________direction_______________________________________________________________ = block.getFieldValue('
													direction
												
                        
                        ');
  // TODO: Assemble JavaScript into code variable.
  var code = '...;\n';
  return code;
};