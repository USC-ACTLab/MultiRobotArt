/**
 * The idea of this file is to contain all of the JavaScript commands that
 * will exported by the Blockly JavaScript exporter, and be evaluated in
 * the actual JavaScript context.
 *
 * Note: Due to scope issues, they _may_ have to be specifically defined locally in the context of the `eval` function.
 */

import {useSimulator} from '@MRAControl/state/useSimulator';
import {Vector3, Color} from 'three';

export type SimulatorGroupState = {
	robotIDs: string[];
};

export const toRadians = (degrees: number): number => {
	return degrees / 180 * Math.PI;
};

export const goToXyzSpeed = (groupState: SimulatorGroupState, x: number, y: number, z: number, speed: number) => {
	var duration = 0;
	groupState.robotIDs.forEach((robotId) => {
		var currPosition;
		if (useSimulator.getState().robots[robotId] == undefined) {
			currPosition = new Vector3(0, 0, 0);
		} else {
			var pos = useSimulator.getState().robots[robotId].pos;
			currPosition = new Vector3(pos.x, pos.y, pos.z);
		}

		const newTrajectory = useSimulator.getState().robotGoTo(robotId, new Vector3(x, y, z), new Vector3(0, 0, 0), new Vector3(0, 0, 0));
		var currDuration = currPosition.distanceTo(new Vector3(x, y, z)) / speed;
		duration = Math.max(currDuration, duration);
		useSimulator.getState().updateTrajectory(robotId, newTrajectory, duration);
	});
	return duration;
};

export const setColor = (groupState: SimulatorGroupState, r = 0, g = 0, b = 0) => {
	groupState.robotIDs.forEach((robotID) =>{
		const robot = {...useSimulator.getState().robots[robotID]};
		robot.color = new Color(r, g, b);
		useSimulator.getState().robots[robotID] = robot;
	});
	return 0.1; // There is actually a cost to switching LEDs
};

export const goToXyzDuration = (groupState: SimulatorGroupState, x: number, y: number, z: number, duration: number) =>{
	groupState.robotIDs.forEach((robotId) => {
		const newTrajectory = useSimulator.getState().robotGoTo(robotId, new Vector3(x, y, z), new Vector3(0, 0, 0), new Vector3(0, 0, 0));
		useSimulator.getState().updateTrajectory(robotId, newTrajectory, duration);
	});
	return duration;
};

export const land = (groupState: SimulatorGroupState, height: number, duration: number) => {
	groupState.robotIDs.forEach((robotId) => {
		var currPosition;
		if (useSimulator.getState().robots[robotId] == undefined) {
			currPosition = new Vector3(0, 0, 0);
		} else {
			var pos = useSimulator.getState().robots[robotId].pos;
			currPosition = new Vector3(pos.x, pos.y, pos.z);
		}

		var goalPosition = currPosition;
		goalPosition.z = height;

		const newTrajectory = useSimulator.getState().robotGoTo(robotId, goalPosition, new Vector3(0, 0, 0), new Vector3(0, 0, 0));
		useSimulator.getState().updateTrajectory(robotId, newTrajectory, duration);
	});
	return duration;
};

export const takeoff = (groupState: SimulatorGroupState, height: number, duration: number) => {
	groupState.robotIDs.forEach((robotId) => {
		var currPosition;
		if (useSimulator.getState().robots[robotId] == undefined) {
			currPosition = new Vector3(0, 0, 0);
		} else {
			var pos = useSimulator.getState().robots[robotId].pos;
			currPosition = new Vector3(pos.x, pos.y, pos.z);
		}

		var goalPosition = currPosition;
		goalPosition.z = height;
		const newTrajectory = useSimulator.getState().robotGoTo(robotId, goalPosition, new Vector3(0, 0, 0), new Vector3(0, 0, 0));
		useSimulator.getState().updateTrajectory(robotId, newTrajectory, duration);
	});
	return duration;
};

export const moveSpeed = (groupState: SimulatorGroupState, x: number, y: number, z: number, speed: number) =>{
	var duration = 0;
	groupState.robotIDs.forEach((robotId) => {
		var currPosition;
		if (useSimulator.getState().robots[robotId] == undefined) {
			currPosition = new Vector3(0, 0, 0);
		} else {
			var pos = useSimulator.getState().robots[robotId].pos;
			currPosition = new Vector3(pos.x, pos.y, pos.z);
		}

		var goalPosition = currPosition;
		goalPosition.x += x;
		goalPosition.y += y;
		goalPosition.z += z;
		const newTrajectory = useSimulator.getState().robotGoTo(robotId, goalPosition, new Vector3(0, 0, 0), new Vector3(0, 0, 0));
		const currDuration = new Vector3(x, y, z).length() / speed; 
		duration = Math.max(currDuration, duration);
		useSimulator.getState().updateTrajectory(robotId, newTrajectory, duration);
	});
	return duration;
};

export const moveCircleVel = (groupState: SimulatorGroupState, radius: number, velocity: number, degrees: number, direction: any) =>{
	console.log(direction); // TODO Add direction?
	var duration = 0;
	groupState.robotIDs.forEach((robotId) =>{
		const axes = ['X', 'Z'];
		const radians = toRadians(degrees);
		const clockwise = false;
		const arclength = radius * (radians);
		duration = arclength / velocity;
		const circleTraj = useSimulator.getState().robotCircle(robotId, radius, axes, radians, clockwise);
		useSimulator.getState().updateTrajectory(robotId, circleTraj, duration);
	});
	return duration;
};


export const dummy = () => {
	return 0.1;
}; // non-zero duration so it's not hidden
