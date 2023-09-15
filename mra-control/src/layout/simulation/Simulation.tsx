import { Crazyflie } from '@MRAControl/components/vector/Crazyflie';
import { useRobartState } from '@MRAControl/state/useRobartState';
import { useSimulator } from '@MRAControl/state/useSimulator';
import { GizmoHelper, GizmoViewport, Grid, OrbitControls, Plane, Sphere } from '@react-three/drei';
import { useFrame } from '@react-three/fiber';
import { useEffect, useRef } from 'react';
import { Group } from 'three';
import * as THREE from 'three'

export const Simulation = () => {
  const marker = useRef<Group>(null!);
  const robots = useSimulator((state) => state.robots);
  const robartRobots = useRobartState((state) => state.robots)
  const step = useSimulator((state) => state.step);
  const setRobots = useSimulator((state) => state.setRobots);
  const robartState = useRobartState();
  const simulatorState = useSimulator();
 
  useFrame(({ clock }) => {
    step();
  });
 
  if (Object.keys(robots).length !== 0){
    if (simulatorState.time === 0){
      Object.values(robots).forEach((robot) => {
        robot.pos.set(...robartState.robots[robot.id].startingPosition);
      });
    }
    else{
    }

  }
  else{
    if (Object.keys(robartRobots).length !== 0){
      setRobots(robartRobots);
    }
  }


  return (
    <>
      <color attach="background" args={['black']} />
      <OrbitControls maxPolarAngle={Math.PI * (1 / 2 - 1 / 20)} minPolarAngle={0} minDistance={5} maxDistance={20} />
      <GizmoHelper alignment="bottom-right" margin={[80, 80]}>
        <GizmoViewport labels={['X', 'Z', 'Y']} axisColors={['#9d4b4b', '#2f7f4f', '#3b5b9d']} labelColor="white" />
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
      <Plane args={[1000, 1000]} rotation={[-Math.PI / 2, 0, 0]} position={[0, -0.02, 0]}>
        <meshStandardMaterial color="black" />
      </Plane>
      {Object.values(robots).map((robot) => (
        <group key={robot.id} ref={marker} position={robot.pos}>
          <Crazyflie robotId={robot.id} renderBoundingBox />
        </group>
      ))}
    </>
  );
};
