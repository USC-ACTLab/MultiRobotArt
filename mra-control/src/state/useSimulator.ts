import { create } from "zustand";
import * as THREE from "three";
import { RobotState } from "./useRobartState";
import { immer } from "zustand/middleware/immer";

export const FPS = 30;

type Trajectory =
  | [
      THREE.Vector3,
      THREE.Vector3,
      THREE.Vector3,
      THREE.Vector3,
      THREE.Vector3,
      THREE.Vector3,
      THREE.Vector3,
      THREE.Vector3
    ]
  | null;

export interface RobotSimState {
  pos: THREE.Vector3;
  trajectory: Trajectory;
  trajectoryDuration: number;
  timeAlongTrajectory: number;
}

export interface SimulatorState {
  robots: Record<string, RobotSimState>;
  time: number;
  timeDilation: number;
  status: "RUNNING" | "STOPPED" | "PAUSED";
}

const defaultSimulatorState: SimulatorState = {
  robots: {},
  time: 0,
  timeDilation: 1,
  status: "RUNNING",
};

export interface SimulatorActions {
  play: () => void;
  pause: () => void;
  halt: () => void;
  step: () => void;
  /**
   * Can only be used when simulator is STOPPED mode.
   * @param robots
   * @returns
   */
  setRobots: (robots: Record<string, RobotState>) => void;
  updateTrajectory: (
    robotId: string,
    trajectory: Trajectory,
    duration: number
  ) => void;
  robotGoTo: (
    robotId: string,
    position: THREE.Vector3,
    velocity: THREE.Vector3,
    acceleration: THREE.Vector3
  ) => void;
}

export const useSimulator = create<SimulatorState & SimulatorActions>()(
  immer((set, get) => ({
    ...defaultSimulatorState,
    play: () => set({ status: "RUNNING" }),
    pause: () => set({ status: "PAUSED" }),
    halt: () => set({ status: "STOPPED" }),
    step: () => {
      const { status, time, timeDilation, robots } = get();
      if (status !== "RUNNING") return;
      const deltaT = 1 / (FPS * timeDilation);
      const newSimTime = time + deltaT;

      Object.keys(robots).forEach((robotId) => {
        const robot = robots[robotId];
        if (robot.trajectory === null) return;

        const newPos = new THREE.Vector3();
        const trajectoryTime = robot.timeAlongTrajectory + deltaT;
        robot.trajectory.map((coefficient, i) => {
          newPos.addScaledVector(coefficient, Math.pow(trajectoryTime, i));
        });

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
          pos: new THREE.Vector3(...robot.startingPosition),
          timeAlongTrajectory: 0,
          trajectory: null,
          trajectoryDuration: 0,
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
      // Solve for the thing that we computed
    },
  }))
);
