/**
 * The idea of this file is to contain all of the JavaScript commands that
 * will exported by the Blockly JavaScript exporter, and be evaluated in
 * the actual JavaScript context.
 *
 * Note: Due to scope issues, they _may_ have to be specifically defined locally in the context of the `eval` function.
 */

import {useSimulator} from '@MRAControl/state/useSimulator';
import {Vector3, Color} from 'three';
import {ComponentTrajectory, Hover, NullTrajectory, type Trajectory} from './trajectories';

export type SimulatorGroupState = {
	robotIDs: string[];
};

export const toRadians = (degrees: number): number => {
	return degrees / 180 * Math.PI;
};

export const goToXyzSpeed = (groupState: SimulatorGroupState, x: number, y: number, z: number, speed: number) => {
	var duration = 0;
	let trajectories: Map<string, Trajectory> = new Map<string, Trajectory>;
	groupState.robotIDs.forEach((robotId) => {
		var currPosition;
		if (useSimulator.getState().robots[robotId] == undefined) {
			currPosition = new Vector3(0, 0, 0);
		} else {
			var pos = useSimulator.getState().robots[robotId].pos;
			currPosition = new Vector3(pos.x, pos.y, pos.z);
		}

		var currDuration = currPosition.distanceTo(new Vector3(x, y, z)) / speed;
		duration = Math.max(currDuration, duration);
		const newTrajectory = useSimulator.getState().robotGoTo(robotId, new Vector3(x, y, z), new Vector3(0, 0, 0), new Vector3(0, 0, 0), currDuration);
		//useSimulator.getState().updateTrajectory(robotId, newTrajectory, duration);
		trajectories.set(robotId, newTrajectory);
	});
	return [duration, trajectories];
};

export const setColor = (groupState: SimulatorGroupState, r = 0, g = 0, b = 0): [number, Map<string, Trajectory>] => {
	let trajectories = new Map<string, Trajectory>();
	groupState.robotIDs.forEach((robotID) =>{
		let robot = {...useSimulator.getState().robots[robotID]};
		// TODO Fix Colors! Need this line to work again and need trajectory to just hover. This will have issues with computing duration...
		robot.color = new Color(r, g, b);
		console.warn(robotID, robot.color);
		useSimulator.setState({
			...useSimulator.getState(),
			robots: {
				...useSimulator.getState().robots,
				[robotID]: robot,
			},
		});
		console.warn(useSimulator.getState().robots[robotID].color)
		trajectories.set(robotID, new Hover(robot.pos));
	});
	return [0.1, trajectories]; // There is actually a cost to switching LEDs
};

export const goToXyzDuration = (groupState: SimulatorGroupState, x: number, y: number, z: number, duration: number) =>{
	let trajectories: Map<string, Trajectory> = new Map<string, Trajectory>;
	groupState.robotIDs.forEach((robotId) => {
		const newTrajectory = useSimulator.getState().robotGoTo(robotId, new Vector3(x, y, z), new Vector3(0, 0, 0), new Vector3(0, 0, 0), duration);
		trajectories.set(robotId, newTrajectory);
		// useSimulator.getState().updateTrajectory(robotId, newTrajectory, duration);
	});
	return [duration, trajectories];
};

export const land = (groupState: SimulatorGroupState, height: number, duration: number) => {
	let trajectories: Map<string, Trajectory> = new Map<string, Trajectory>;
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

		const newTrajectory = useSimulator.getState().robotGoTo(robotId, goalPosition, new Vector3(0, 0, 0), new Vector3(0, 0, 0), duration);
		//useSimulator.getState().updateTrajectory(robotId, newTrajectory, duration);
		trajectories.set(robotId, newTrajectory);
	});
	return [duration, trajectories];
};

export const takeoff = (groupState: SimulatorGroupState, height: number, duration: number) => {
	let trajectories: Map<string, Trajectory> = new Map<string, Trajectory>;
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
		const newTrajectory = useSimulator.getState().robotGoTo(robotId, goalPosition, new Vector3(0, 0, 0), new Vector3(0, 0, 0), duration);
		//useSimulator.getState().updateTrajectory(robotId, newTrajectory, duration);
		trajectories.set(robotId, newTrajectory);
	});
	return [duration, trajectories];
};

export const moveSpeed = (groupState: SimulatorGroupState, x: number, y: number, z: number, speed: number) =>{
	var duration = 0;
	let trajectories: Map<string, Trajectory> = new Map<string, Trajectory>;
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
		const currDuration = new Vector3(x, y, z).length() / speed; 
		const newTrajectory = useSimulator.getState().robotGoTo(robotId, goalPosition, new Vector3(0, 0, 0), new Vector3(0, 0, 0), currDuration);
		duration = Math.max(currDuration, duration);
		trajectories.set(robotId, newTrajectory);
		//useSimulator.getState().updateTrajectory(robotId, newTrajectory, duration);
	});
	return [duration, trajectories];
};

export const moveCircleVel = (groupState: SimulatorGroupState, radius: number, velocity: number, degrees: number, direction: any): [number, Map<string, Trajectory>] =>{
	console.log(direction); // TODO Add direction?
	var duration = 0;
	let trajectories: Map<string, Trajectory> = new Map<string, Trajectory>;
	groupState.robotIDs.forEach((robotId) =>{
		const axes = ['X', 'Z'];
		const radians = toRadians(degrees);
		const clockwise = false;
		const arclength = radius * (radians);
		duration = arclength / velocity;
		const circleTraj = useSimulator.getState().robotCircle(robotId, radius, axes, radians, clockwise);
		trajectories.set(robotId, circleTraj);
		// useSimulator.getState().updateTrajectory(robotId, circleTraj, duration);
	});
	return [duration, trajectories];
};


export const dummy = () => {
	return [0.1, new NullTrajectory()];
}; // non-zero duration so it's not hidden

export const componentTraj = (groupState: SimulatorGroupState, [durX, trajX]: [number, Map<string, Trajectory>],
	[durY, trajY]: [number, Map<string, Trajectory>], [durZ, trajZ]: [number, Map<string, Trajectory>]): [number, Map<string, Trajectory>] => {
	let duration = Math.max(durX, durY, durZ);
	let trajectories: Map<string, Trajectory> = new Map<string, Trajectory>;
	groupState.robotIDs.forEach((robotId) =>{
		let trajectory = new ComponentTrajectory(duration, durX, trajX.get(robotId), durY, trajY.get(robotId), durZ, trajZ.get(robotId));
		trajectories.set(robotId, trajectory); 
	});
	return [duration, trajectories];
};