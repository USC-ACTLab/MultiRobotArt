/**
 * The idea of this file is to contain all of the JavaScript commands that
 * will exported by the Blockly JavaScript exporter, and be evaluated in
 * the actual JavaScript context.
 *
 * Note: Due to scope issues, they _may_ have to be specifically defined locally in the context of the `eval` function.
 */

import { useSimulator } from '@MRAControl/state/useSimulator';
import { Vector3 } from 'three';

export interface SimulatorGroupState {
  robotIDs: string[];
}

export const go_to_xyz = (groupState: SimulatorGroupState, x: number, y: number, z: number, speed: number) => {
  groupState.robotIDs.forEach((robotId) => {
    const newTrajectory = useSimulator.getState().robotGoTo(robotId, new Vector3(x, y, z), new Vector3(0, 0, 0), new Vector3(0, 0, 0));
    useSimulator.getState().updateTrajectory(robotId, newTrajectory, speed);
  });
};

export const dummy = () => {};
