import { Crazyflie } from '@MRAControl/components/vector/Crazyflie';
import { useRobartState } from '@MRAControl/state/useRobartState';
import { useSimulator } from '@MRAControl/state/useSimulator';
import { Environment, GizmoHelper, GizmoViewport, Grid, OrbitControls, Plane, Sky, Sphere } from '@react-three/drei';
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

  // useEffect(() => {
  //   setRobots(useRobartState.getState().robots);
  // }, [robots]);

  useFrame(({ clock }) => {
    step();
  });

  return (
    <>
     <color attach="background" args={['black']} />
      <OrbitControls maxPolarAngle={Math.PI * (1/2 - 1 / 20)} minPolarAngle={0} minDistance={5} maxDistance={20} />
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
      <Plane args={[1000, 1000]} rotation={[-Math.PI/2, 0, 0]} position={[0,-0.02, 0]}>
        <meshStandardMaterial color="black" />
      </Plane>
      {Object.values(robots).map((robot) => (
        <group key={robot.id} ref={marker} position={robot.pos}>
          <Crazyflie />
          <Sphere position={[0,-0.05,0]} scale={0.1} castShadow={false} receiveShadow={false}>
            <meshDistanceMaterial />
            <pointLight position={[0,0,0]} intensity={1} color="white"/>
          </Sphere>
          
        </group>
      ))}
    </>
  );
};
