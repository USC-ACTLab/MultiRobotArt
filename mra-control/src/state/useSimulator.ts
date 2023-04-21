import * as THREE from 'three';
import { create } from 'zustand';
import { immer } from 'zustand/middleware/immer';

import { RobotState, TimelineState, useRobartState } from './useRobartState';
import { SimulatorGroupState } from './simulatorCommands';
import * as SIM from './simulatorCommands';

export const FPS = 60;

type TrajectoryPolynomial = [THREE.Vector3, THREE.Vector3, THREE.Vector3, THREE.Vector3, THREE.Vector3, THREE.Vector3, THREE.Vector3, THREE.Vector3] | null;

export interface Trajectory {
  polynomial: TrajectoryPolynomial;
  duration: number;
}

export interface RobotSimState {
  id: string;
  pos: THREE.Vector3;
  vel: THREE.Vector3;
  acc: THREE.Vector3;
  trajectories: Trajectory[];
  trajectory: TrajectoryPolynomial;
  trajectoryDuration: number;
  timeAlongTrajectory: number;
}

export interface SimulatorState {
  robots: Record<string, RobotSimState>;
  time: number;
  timeDilation: number;
  status: 'RUNNING' | 'STOPPED' | 'PAUSED';
}

const defaultSimulatorState: SimulatorState = {
  robots: {},
  time: 0,
  timeDilation: 1,
  status: 'STOPPED',
};

const SIMULATOR_TIMEOUTS: NodeJS.Timeout[] = [];

export interface SimulatorActions {
  play: () => void;
  pause: () => void;
  resume: () => void;
  halt: () => void;
  step: () => void;
  /**
   * Can only be used when simulator is STOPPED mode.
   * @param robots
   * @returns
   */
  setRobots: (robots: Record<string, RobotState>) => void;
  updateTrajectory: (robotId: string, trajectory: TrajectoryPolynomial, duration: number) => void;
  robotGoTo: (robotId: string, position: THREE.Vector3, velocity: THREE.Vector3, acceleration: THREE.Vector3) => TrajectoryPolynomial;
  executeSimulation: (startTime: number) => void;
  cancelSimulation: () => void;
}

export const useSimulator = create<SimulatorState & SimulatorActions>()(
  immer((set, get) => ({
    ...defaultSimulatorState,
    play: () => { 
      set({ status: 'RUNNING', time: 0 });
      get().executeSimulation(0);
   },
    pause: () => {
      set({ status: 'PAUSED' });
      get().cancelSimulation();
    },
    resume: () => {
      set({ status: 'RUNNING' });
      get().executeSimulation(get().time);
    },
    halt: () => {
      set({ status: 'STOPPED' });
      get().cancelSimulation();
    },
    step: () => {
      const { status, time, timeDilation, robots: currentRobots } = get();
      if (status !== 'RUNNING') return;
      const deltaT = 1 / (FPS * timeDilation);
      const newSimTime = time + deltaT;
      const robots = { ...currentRobots };

      Object.keys(robots).forEach((robotId) => {
        const robot = robots[robotId];
        if (robot.trajectory === null) return;

        const newPos = new THREE.Vector3();
        const trajectoryTime = robot.timeAlongTrajectory + deltaT / robot.trajectoryDuration;
        robot.trajectory.map((coefficient, i) => {
          newPos.addScaledVector(coefficient, Math.pow(trajectoryTime, i));
        });

        console.log('robot', robotId, 'newpos', newPos);

        robots[robotId] = {
          ...robots[robotId],
          pos: newPos,
          timeAlongTrajectory: trajectoryTime,
        };

        if (robots[robotId].timeAlongTrajectory >= 1) {
          robots[robotId].trajectory = null;
          robots[robotId].timeAlongTrajectory = 0;
        }
      });

      set({
        robots,
        time: newSimTime,
      });
    },
    setRobots: (robots) => {
      const simRobots: Record<string, RobotSimState> = {};
      Object.values(robots).forEach((robot) => {
        simRobots[robot.id] = {
          id: robot.id,
          pos: new THREE.Vector3(...robot.startingPosition),
          vel: new THREE.Vector3(),
          acc: new THREE.Vector3(),
          timeAlongTrajectory: 0,
          trajectory: null,
          trajectoryDuration: 0,
          trajectories: []
        };
      });
      set({ robots: simRobots });
    },
    updateTrajectory: (robotId, trajectory, duration) => {
      set((state) => {
        state.robots[robotId].timeAlongTrajectory = 0;
        state.robots[robotId].trajectory = trajectory;
        state.robots[robotId].trajectoryDuration = duration;
      });
    },
    robotGoTo: (robotId, pos, vel, acc) => {
      const robot = get().robots[robotId];

      // Degree 7 Polynomial solution to IVP
      const a0 = robot.pos.clone();
      const a1 = robot.vel.clone();
      const a2 = robot.acc.clone().multiplyScalar(0.5);
      const a3 = new THREE.Vector3();
      const a4 = robot.acc
        .clone()
        .multiplyScalar(2)
        .addScaledVector(acc, -1)
        .addScaledVector(robot.vel, 8)
        .addScaledVector(vel, 6)
        .addScaledVector(robot.pos, 14)
        .addScaledVector(pos, -14)
        .multiplyScalar(-2.5);
      const a5 = robot.acc
        .clone()
        .multiplyScalar(10)
        .addScaledVector(acc, -7)
        .addScaledVector(robot.vel, 45)
        .addScaledVector(vel, 39)
        .addScaledVector(robot.pos, 84)
        .addScaledVector(pos, -84);
      const a6 = robot.acc
        .clone()
        .multiplyScalar(-15)
        .addScaledVector(acc, 13)
        .addScaledVector(robot.vel, -72)
        .addScaledVector(vel, -68)
        .addScaledVector(robot.pos, -140)
        .addScaledVector(pos, 140)
        .multiplyScalar(0.5);
      const a7 = robot.acc
        .clone()
        .addScaledVector(acc, -1)
        .addScaledVector(robot.vel, 5)
        .addScaledVector(vel, 5)
        .addScaledVector(robot.pos, 10)
        .addScaledVector(pos, -10)
        .multiplyScalar(2);

      return [a0, a1, a2, a3, a4, a5, a6, a7] as any; // TODO: For some reason typescript is unhappy here...
    },
    executeSimulation: (startTime) => {
      const timeline = useRobartState.getState().timelineState;
      const blocks = useRobartState.getState().blocks;
      console.log('execute', timeline.groups);
      Object.values(timeline.groups).forEach(group => {
        // Need the following local variables so that the EVAL works properly.
        const group_state: SimulatorGroupState = {
          robotIDs: Object.keys(group.robots)
        };
        const simulator = SIM; // This is the simulator object for commands, necessary for the eval to work.
        // END: The need of said local variables
        Object.values(group.items).forEach(timelineItem => {
          console.log('item time', timelineItem.startTime);
          const offset = timelineItem.startTime - startTime;
          if(offset < 0) return;
          
          const timeout = setTimeout(() => {
            eval(blocks[timelineItem.blockId].javaScript); // TODO: Totally safe, no security flaws whatsoever.
          }, timeline.scale * offset * 1000);
          SIMULATOR_TIMEOUTS.push(timeout);
        });
      })
    },
    cancelSimulation: () => {
      // Clear all of the timeouts
      SIMULATOR_TIMEOUTS.map(timeout => clearInterval(timeout));
      while(SIMULATOR_TIMEOUTS.length > 0) 
        SIMULATOR_TIMEOUTS.pop();
    }
  })),
);
