
import * as THREE from 'three';

import {create} from 'zustand';
import {subscribeWithSelector} from 'zustand/middleware';
import {immer} from 'zustand/middleware/immer';
import {useRobartState} from './useRobartState';

type ConstraintViolation = 'velocity' | 'acceleration' | 'workspace';

export type ConstraintWarning = {
	time: number;
	repr: string;
	violationType: ConstraintViolation;
	robotId: string;
};

export type DynamicConstraintState = {
	maxVelocity: number;
	maxAcceleration: number;
	deltaT: number;
	setMaxVelocity: (vel: number) => void;
	setMaxAcceleration: (acc: number) => void;
};

export type KinematicConstraintState = {
	workspaceDimensions: THREE.Box3;
	setWorkspaceDimensions: (dim: THREE.Box3) => void;
};

export type ConstraintChecker = {
	checkConstraints: (robotIDs: string[]) => ConstraintWarning[] | undefined;
	checkKinematicConstraints: (robotIDs: string[]) => ConstraintWarning[] | undefined;
	checkDynamicConstraints: (robotIDs: string[]) => ConstraintWarning[] | undefined;
	positionHistory: Map<number, Map<string, THREE.Vector3>>;
};

export type ConstraintState = DynamicConstraintState & KinematicConstraintState & ConstraintChecker;

export const useCrazyflieConstraintState = create<ConstraintState>()(
	subscribeWithSelector(
		immer((set, get) => ({
			maxVelocity: 1.0,
			maxAcceleration: 5.0,
			workspaceDimensions: new THREE.Box3(new THREE.Vector3(-4, -2.5, -0.01), new THREE.Vector3(2, 2.5, 2.5)),
			positionHistory: new Map<number, Map<string, THREE.Vector3>>(),
			deltaT: 1 / 60,
			setMaxVelocity(vel) {
				set({
					maxVelocity: vel,
				});
			},
			setMaxAcceleration(acc) {
				set({
					maxAcceleration: acc,
				});
			},
			setWorkspaceDimensions(dim) {
				set({
					workspaceDimensions: dim,
				});
			},
			checkDynamicConstraints(robotIDs: string[]) {
				const positions = get().positionHistory;
				if (positions.size === 0) {
					return undefined;
				}

				// Grab robot IDs
				const timesteps = Array.from(positions.keys());
				const sortedTimesteps = timesteps.sort((a, b) => a < b ? -1 : a > b ? 1 : 0);

				let warnings: ConstraintWarning[] = [];
				//Check velocity Constraints
				robotIDs.forEach(id => {
					// For Each robot Check if currPos - prevPos > maxVel
					for (let i = 1; i < sortedTimesteps.length; i++) {
						const currentPosition = positions.get(sortedTimesteps[i])?.get(id);
						const previousPosition = positions.get(sortedTimesteps[i - 1])?.get(id);
						if (currentPosition && previousPosition) {
							const velocity = currentPosition.distanceTo(previousPosition) / get().deltaT;
							if (currentPosition && previousPosition &&  velocity > get().maxVelocity) {
								const robotName = useRobartState.getState().robots[id].name;
								warnings.push({
									time: sortedTimesteps[i],
									repr: 'robot ' + robotName + ' has violated a velocity constraint at time ' + sortedTimesteps[i].toFixed(2) + '. It was travelling at ' + velocity.toFixed(2) + ' m/s.\n',
									violationType: 'velocity',
									robotId: id,
								});
							}
						}
					}
				});
                
				// Check Acceleration Constraints

				return warnings;
			},
			checkConstraints(robotIDs: string[]) {
				const dynamicConstraints = this.checkDynamicConstraints(robotIDs);
				const kinematicConstraints = this.checkKinematicConstraints(robotIDs);
				if (kinematicConstraints === undefined)
					return dynamicConstraints;
				return dynamicConstraints?.concat(kinematicConstraints);
			},
			checkKinematicConstraints(robotIDs: string[]) {
				const history = get().positionHistory;
				if (history.size > 0) {
					let warnings: ConstraintWarning[] = [];
					const timesteps = Array.from(history.keys());

					const sortedTimesteps = timesteps.sort((a, b) => a < b ? -1 : a > b ? 1 : 0);
					// Check workspace bounds
					robotIDs.forEach(id => {
					// For Each robot Check if currPos - prevPos > maxVel
						for (let i = 1; i < sortedTimesteps.length; i++) {
							const currentPosition = history.get(sortedTimesteps[i])?.get(id);
							if (currentPosition) {
								if (!this.workspaceDimensions.containsPoint(currentPosition)) {
									const robotName = useRobartState.getState().robots[id].name;
									warnings.push({
										time: sortedTimesteps[i],
										repr: 'robot ' + robotName + ' has violated a workspace constraint at time ' + sortedTimesteps[i].toFixed(2) + '. It\'s position was ' + currentPosition.x.toFixed(2) + ', ' + currentPosition.y.toFixed(2) + ', ' + currentPosition.z.toFixed(2) + '\n',
										violationType: 'velocity',
										robotId: id,
									});
								}

							}
						}
					});
					return warnings;
				}

				return undefined;           
			},
		})),
	),
);