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
    vel: THREE.Vector3;
    acc: THREE.Vector3;
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
                    newPos.addScaledVector(
                        coefficient,
                        Math.pow(trajectoryTime, i)
                    );
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
                    vel: new THREE.Vector3(),
                    acc: new THREE.Vector3(),
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
            const robot = get().robots[robotId];

            // Degree 7 Polynomial solution to IVP
            const a0 = robot.pos;
            const a1 = robot.vel;
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
                .addScaledVector(pos, -1)
                .multiplyScalar(2);

            set(
                (state) =>
                    (state.robots[robotId].trajectory = [
                        a0,
                        a1,
                        a2,
                        a3,
                        a4,
                        a5,
                        a6,
                        a7,
                    ])
            );
        },
    }))
);
