import { Crazyflie } from '@MRAControl/components/vector/Crazyflie';
import { useRobartState } from '@MRAControl/state/useRobartState';
import { useSimulator } from '@MRAControl/state/useSimulator';
import { Environment, GizmoHelper, GizmoViewport, Grid, OrbitControls, Sky, Sphere } from '@react-three/drei';
import { useFrame } from '@react-three/fiber';
import { useEffect, useRef } from 'react';
import { Group } from 'three';
import * as THREE from 'three';

export const Simulation = () => {
  const marker = useRef<Group>(null!);
  const robots = useSimulator((state) => state.robots);
  const updateTrajectory = useSimulator((state) => state.updateTrajectory);
  const robotGoTo = useSimulator((state) => state.robotGoTo);
  const setRobots = useSimulator((state) => state.setRobots);
  const step = useSimulator((state) => state.step);

  useEffect(() => {
    setRobots(useRobartState.getState().robots);
    setInterval(() => {
      Object.values(robots).map((robot) => {
        const trajectory = robotGoTo(
          robot.id,
          new THREE.Vector3(Math.random() * 5, Math.random() * 5, Math.random() * 5),
          new THREE.Vector3(),
          new THREE.Vector3(),
        );
        updateTrajectory(robot.id, trajectory, Math.random() * 8 + 2);
      });
    }, 5 * 1000);

    // console.log(robots);
  }, []);

  useFrame(({ clock }) => {
    step();
    console.log(robots);
  });

  return (
    <>
      <OrbitControls maxPolarAngle={Math.PI * (1 / 2 - 1 / 10)} minPolarAngle={0} minDistance={15} maxDistance={20} />
      <Environment preset="night" background={true} />
      <GizmoHelper alignment="bottom-right" margin={[80, 80]}>
        <GizmoViewport axisColors={['#9d4b4b', '#2f7f4f', '#3b5b9d']} labelColor="white" />
      </GizmoHelper>
      <Grid
        position={[0, -0.001, 0]}
        args={[10.5, 10.5]}
        cellSize={1}
        cellThickness={1}
        cellColor={'black'}
        fadeDistance={50}
        fadeStrength={1.2}
        infiniteGrid={true}
      />
      {/* <ambientLight intensity={0.1} />
      <directionalLight color="blue" position={[0, 5, 5]} /> */}
      {/* <Sky
                distance={450000}
                sunPosition={[0, 1, 0]}
                inclination={0}
                azimuth={0.25}
            /> */}

      {Object.values(robots).map((robot) => (
        <group key={robot.id} ref={marker} position={robot.pos}>
          <Crazyflie />
          <pointLight intensity={1} />
        </group>
      ))}
    </>
  );
};
