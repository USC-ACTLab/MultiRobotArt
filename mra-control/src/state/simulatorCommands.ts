/**
 * The idea of this file is to contain all of the JavaScript commands that
 * will exported by the Blockly JavaScript exporter, and be evaluated in
 * the actual JavaScript context.
 *
 * Note: Due to scope issues, they _may_ have to be specifically defined locally in the context of the `eval` function.
 */

import { RobotSidebar } from '@MRAControl/layout/robotManager/RobotSidebar';
import { useSimulator } from '@MRAControl/state/useSimulator';
import { Vector3, Color } from 'three';

export interface SimulatorGroupState {
  robotIDs: string[];
}

export const toRadians = (degrees: number): number => {
  return degrees / 180 * Math.PI;
}

export const go_to_xyz_speed = (groupState: SimulatorGroupState, x: number, y: number, z: number, speed: number) => {
  var duration = 0;
  groupState.robotIDs.forEach((robotId) => {
    var curr_position;
    if (useSimulator.getState().robots[robotId] == undefined){
      curr_position = new Vector3(0, 0, 0)
    }
    else{
      var pos = useSimulator.getState().robots[robotId].pos;
      curr_position = new Vector3(pos.x, pos.y, pos.z);
    }
    const newTrajectory = useSimulator.getState().robotGoTo(robotId, new Vector3(x, y, z), new Vector3(0, 0, 0), new Vector3(0, 0, 0));
    var curr_duration = curr_position.distanceTo(new Vector3(x, y, z)) / speed;
    duration = Math.max(curr_duration, duration)
    useSimulator.getState().updateTrajectory(robotId, newTrajectory, duration);
  });
  return duration;
};

export const setColor = (groupState: SimulatorGroupState, r: number, g: number, b: number) => {
  groupState.robotIDs.forEach((robotID) =>{
    const robot = {...useSimulator.getState().robots[robotID]}
    robot.color = new Color(r, g, b);
    useSimulator.getState().robots[robotID] = robot;
  });
  return 0.1; // There is actually a cost to switching LEDs
};

export const go_to_xyz_duration = (groupState: SimulatorGroupState, x: number, y: number, z: number, duration: number) =>{
  groupState.robotIDs.forEach((robotId) => {
    const newTrajectory = useSimulator.getState().robotGoTo(robotId, new Vector3(x, y, z), new Vector3(0, 0, 0), new Vector3(0, 0, 0));
    useSimulator.getState().updateTrajectory(robotId, newTrajectory, duration)
  });
  return duration;
};

export const land = (groupState: SimulatorGroupState, height: number, duration: number) => {
  groupState.robotIDs.forEach((robotId) => {
    var curr_position
    if (useSimulator.getState().robots[robotId] == undefined){
      curr_position = new Vector3(0, 0, 0)
    }
    else{
      var pos = useSimulator.getState().robots[robotId].pos;
      curr_position = new Vector3(pos.x, pos.y, pos.z);
    }    var goal_position = curr_position
    goal_position.z = height

    const newTrajectory = useSimulator.getState().robotGoTo(robotId, goal_position, new Vector3(0, 0, 0), new Vector3(0, 0, 0));
    useSimulator.getState().updateTrajectory(robotId, newTrajectory, duration)
  });
  return duration;
};

export const takeoff = (groupState: SimulatorGroupState, height: number, duration: number) => {
  groupState.robotIDs.forEach((robotId) => {
    var curr_position;
    if (useSimulator.getState().robots[robotId] == undefined){
      curr_position = new Vector3(0, 0, 0)
    }
    else{
      var pos = useSimulator.getState().robots[robotId].pos;
      curr_position = new Vector3(pos.x, pos.y, pos.z);
    }
    var goal_position = curr_position;
    goal_position.z = height;
    const newTrajectory = useSimulator.getState().robotGoTo(robotId, goal_position, new Vector3(0, 0, 0), new Vector3(0, 0, 0));
    useSimulator.getState().updateTrajectory(robotId, newTrajectory, duration)
  });
  return duration
};

export const move_speed = (groupState: SimulatorGroupState, x: number, y: number, z: number, speed: number) =>{
  var duration = 0;
  groupState.robotIDs.forEach((robotId) => {
    var curr_position;
    if (useSimulator.getState().robots[robotId] == undefined){
      curr_position = new Vector3(0, 0, 0)
    }
    else{
      var pos = useSimulator.getState().robots[robotId].pos;
      curr_position = new Vector3(pos.x, pos.y, pos.z);
    }
    var goal_position = curr_position;
    goal_position.x += x;
    goal_position.y += y;
    goal_position.z += z;
    const newTrajectory = useSimulator.getState().robotGoTo(robotId, goal_position, new Vector3(0, 0, 0), new Vector3(0, 0, 0));
    const curr_duration = new Vector3(x, y, z).length() / speed; 
    duration = Math.max(curr_duration, duration);
    useSimulator.getState().updateTrajectory(robotId, newTrajectory, duration)
  });
  return duration;
};

export const move_circle_vel = (groupState: SimulatorGroupState, radius: number, velocity: number, degrees: number, direction: any) =>{
  var duration = 0;
  groupState.robotIDs.forEach((robotId) =>{
    var curr_position;
    if (useSimulator.getState().robots[robotId] == undefined){
      curr_position = new Vector3(0, 0, 0);
    }
    else{
      var pos = useSimulator.getState().robots[robotId].pos;
      curr_position = new Vector3(pos.x, pos.y, pos.z);
    }
    const axes = ['X', 'Z'];
    const radians = toRadians(degrees);
    const clockwise = false;
    // TODO: duration calculation...
    const arclength = radius *(radians);
    duration = arclength / velocity;
    const circle_traj = useSimulator.getState().robotCircle(robotId, radius, axes, radians, clockwise);
    useSimulator.getState().updateTrajectory(robotId, circle_traj, duration);
  });
  console.log("Duration", duration);
  return duration;
};


export const dummy = () => {return 0.1};
