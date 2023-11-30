/* eslint-disable react/no-unknown-property */
/* eslint-disable @typescript-eslint/no-unused-vars */
import {Crazyflie} from '@MRAControl/components/vector/Crazyflie';
import {useRobartState} from '@MRAControl/state/useRobartState';
import {useSimulator} from '@MRAControl/state/useSimulator';
import {CatmullRomLine, GizmoHelper, GizmoViewport, Grid, OrbitControls, Plane, Sphere} from '@react-three/drei';
import {useFrame, useThree} from '@react-three/fiber';
import {useRef} from 'react';
import React from 'react';
import type THREE from 'three';
import {type Group, type Vector3} from 'three';

let init = true;
export const Simulation = () => {
	const marker = useRef<Group>(null!);
	const robots = useSimulator((state) => state.robots);
	const robartRobots = useRobartState((state) => state.robots);
	const step = useSimulator((state) => state.step);
	const setRobots = useSimulator((state) => state.setRobots);
	const robartState = useRobartState();
	const simulatorState = useSimulator();
	const renderBB = simulatorState.renderBoundingBoxes;
 
	const trajectoryMarkers: Array<{position: Vector3; color: THREE.Color; id: string}> = useSimulator((state) => state.trajectoryMarkers);
	useFrame(({clock}) => {
		step();
	});
 
	if (Object.keys(robots).length !== 0) {
		if (simulatorState.time === 0) {
			Object.values(robots).forEach((robot) => {
				robot.pos.set(...robartState.robots[robot.id].startingPosition);
			});
		}

	} else {
		if (Object.keys(robartRobots).length !== 0) {
			setRobots(robartRobots);
		}
	}

	const {size, viewport, camera} = useThree();
	// if (init || camera.up !== new THREE.Vector3(0, 0, 1)) {
	if (init || (camera.up.x !== 0 || camera.up.y !== 0 || camera.up.z !== 1)) {
		camera.position.set(-5, 0, 2);
		camera.up.set(0, 0, 1);
		init = false;
	}

	return (
		<>
			<color attach="background" args={['black']} />
			<OrbitControls  maxPolarAngle={Math.PI * (1 / 2 - 1 / 20)} minPolarAngle={0} minAzimuthAngle={-Math.PI / 2} maxAzimuthAngle={Math.PI / 2} minDistance={.5} maxDistance={5} object={camera}/>
			{/* <FlyControls></FlyControls> */}
			<GizmoHelper alignment="bottom-right" margin={[80, 80]}>
				<GizmoViewport labels={['X', 'Y', 'Z']} axisColors={['#9d4b4b', '#2f7f4f', '#3b5b9d']} labelColor="white" />
			</GizmoHelper>
			<Grid
				position={[0, 0, -0.001]}
				args={[10.5, 10.5]}
				cellSize={1}
				cellThickness={1}
				rotation={[0, -Math.PI / 2, -Math.PI / 2]}
				cellColor={'black'}
				fadeDistance={50}
				fadeStrength={1.2}
				infiniteGrid={true}
			/>
			<Plane args={[1000, 1000]} rotation={[0, 0, -Math.PI / 2]} position={[0, 0, -0.02]}>
				<meshStandardMaterial color="black" />
			</Plane>
			{Object.values(robots).map((robot) => (
				<group key={robot.id} ref={marker} position={robot.pos}>
					robartState
					<Crazyflie robotId={robot.id} renderBoundingBox={renderBB} />
					
				</group>
			))}
			{Object.values(trajectoryMarkers).map((trajectoryMarker) => (
				<group key={trajectoryMarker.id} position={trajectoryMarker.position}>
					<Sphere args={[0.03]}>
						<meshBasicMaterial color={trajectoryMarker.color.getHex()}/> 
					</Sphere>
				</group>
			))}

			<primitive object={camera}></primitive>
		</>
	);
};
