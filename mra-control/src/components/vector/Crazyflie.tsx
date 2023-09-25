/* eslint-disable react/no-unknown-property */
/* eslint-disable @typescript-eslint/no-unsafe-argument */
/* eslint-disable @typescript-eslint/naming-convention */
/* eslint-disable @typescript-eslint/no-unsafe-assignment */
/*
Auto-generated by: https://github.com/pmndrs/gltfjsx
*/
import {useRobartState} from '@MRAControl/state/useRobartState';
import {useSimulator} from '@MRAControl/state/useSimulator';
import {Box, Sphere, Text, useAnimations, useGLTF} from '@react-three/drei';
import {useThree} from '@react-three/fiber';
import React, {useEffect, useRef, useState} from 'react';
import {Box3, MeshBasicMaterial, Vector3} from 'three';

type CrazyflieProps = {
	robotId: string;
	renderBoundingBox: boolean;
};

export function Crazyflie({robotId, renderBoundingBox}: CrazyflieProps) {
	const updateRobotBoundingBox = useSimulator((state) => state.updateRobotBoundingBox);
	const checkCollisions = useSimulator((state) => state.checkCollisions);
	const robot = useRobartState((state) => state.robots[robotId]);
	const simRobot = useSimulator((state) => state.robots[robotId]);
	const boundingBox = simRobot?.boundingBox;
	const boundingBoxDims = boundingBox?.getSize(new Vector3());

	const [showText, setShowText] = useState(false);
	const {camera} = useThree();

	const group = useRef<THREE.Group>(null);
	const {nodes, materials, animations} = useGLTF('./crazyflie.glb') as any;
	const {actions} = useAnimations(animations, group);
	const [isLoadingCFs, setIsLoadingCFs] = useState(true);
	const [isLoadingWireframes, setIsLoadingWireframes] = useState(true);

	useEffect(() => {
		Object.values(actions).map((a) => a?.play());
		setIsLoadingCFs(false);
	}, []);

	useEffect(() => {
		if (!group.current) {
			setIsLoadingWireframes(false);
			return;
		}

		if (renderBoundingBox) {
			const crazyflieBoundingBox = new Box3();
			crazyflieBoundingBox.setFromObject(group.current);
			updateRobotBoundingBox(robotId, crazyflieBoundingBox);
		}

		setIsLoadingWireframes(false);
	}, [group.current]);
	if (isLoadingCFs || isLoadingWireframes) {
		return <Text fontSize={1}>Loading...</Text>;
	}

	return (
		<>
			{showText ? (
				<Text quaternion={camera.quaternion.clone()} position={[0, 0, 1]} fontSize={0.25}>
					{`${robot.name}, Position: (${simRobot.pos.x.toFixed(1)}, ${simRobot.pos.y.toFixed(1)}, ${simRobot.pos.z.toFixed(1)})`}
				</Text>
			) : null}
			<group>
				<group ref={group}>
					<group scale={0.3} rotation={[Math.PI / 2, -Math.PI / 2, 0]}>
						<mesh
							name="pcb_Default_sldprt"
							castShadow
							receiveShadow
							geometry={nodes.pcb_Default_sldprt.geometry}
							material={materials['Material.003']}
							position={[0.01, 0, 0]}
							scale={0.01}
						>
							<mesh
								name="propeller_ccw_Default_sldprt001"
								castShadow
								receiveShadow
								geometry={nodes.propeller_ccw_Default_sldprt001.geometry}
								material={materials['Material.004']}
								position={[33.94, 15.2, 33.85]}
								rotation={[1.56, -0.01, 0.79]}
							/>
							<mesh
								name="propeller_ccw_Default_sldprt002"
								castShadow
								receiveShadow
								geometry={nodes.propeller_ccw_Default_sldprt002.geometry}
								material={materials['Material.001']}
								position={[33.94, 15.2, -33.85]}
								rotation={[-1.56, 0.01, 0.79]}
								scale={-1}
							/>
							<mesh
								name="propeller_ccw_Default_sldprt003"
								castShadow
								receiveShadow
								geometry={nodes.propeller_ccw_Default_sldprt003.geometry}
								material={materials['Material.004']}
								position={[-33.83, 15.2, 33.32]}
								rotation={[-1.56, 0.01, 0.79]}
								scale={-1}
							/>
							<mesh
								name="propeller_ccw_Default_sldprt004"
								castShadow
								receiveShadow
								geometry={nodes.propeller_ccw_Default_sldprt004.geometry}
								material={materials['Material.004']}
								position={[-33.94, 15.2, -33.85]}
								rotation={[1.57, 0, Math.PI / 4]}
							/>
							<mesh
								name="7mm_motor_mount_Default_sldprt"
								castShadow
								receiveShadow
								geometry={nodes['7mm_motor_mount_Default_sldprt'].geometry}
								material={materials.Material}
								position={[-33.72, 3.79, -33.71]}
								rotation={[1.56, 0, 0]}
							/>
							<mesh
								name="7x15_motor_Default_sldprt"
								castShadow
								receiveShadow
								geometry={nodes['7x15_motor_Default_sldprt'].geometry}
								material={materials['Material.002']}
								position={[-33.85, -4.25, -33.74]}
								rotation={[1.56, 0, 0]}
							/>
							<mesh
								name="battery_bc-bl-01-a_Default_sldprt"
								castShadow
								receiveShadow
								geometry={nodes['battery_bc-bl-01-a_Default_sldprt'].geometry}
								material={materials['Material.006']}
								position={[-11.27, 2.54, 8.16]}
								rotation={[1.56, 0, 0]}
							/>
							<mesh
								name="battery_holder_board_Default_sldprt"
								castShadow
								receiveShadow
								geometry={nodes.battery_holder_board_Default_sldprt.geometry}
								material={materials['Material.003']}
								position={[-10.37, 9.9, 13.13]}
								rotation={[Math.PI / 2, 0, 0]}
								scale={[1, 0.97, 1]}
							/>
							<mesh
								name="header_Default_sldprt"
								castShadow
								receiveShadow
								geometry={nodes.header_Default_sldprt.geometry}
								material={materials['Material.005']}
								position={[-9.32, 4.28, -8.95]}
								rotation={[1.56, 0, -Math.PI]}
								scale={-1}
							/>
						</mesh>
						{/* <Sphere position={[0, 0, -0.05]} scale={0.033} castShadow={false} receiveShadow={false}>
            <meshStandardMaterial emissive={simRobot ? [simRobot.color.r / 255, simRobot.color.g / 255, simRobot.color.b / 255] : [1, 1, 1]} />
            <pointLight position={[0, 0, 0]} intensity={0.5} />
          </Sphere> */}
					</group>
					<Sphere position={[0, 0, -0.05]} scale={0.033} castShadow={false} receiveShadow={false}>
						<meshStandardMaterial emissive={simRobot ? [simRobot.color.r / 255, simRobot.color.g / 255, simRobot.color.b / 255] : [1, 1, 1]} />
						<pointLight position={[0, 0, 0]} intensity={0.5} />
					</Sphere>
				</group>
			</group>
			{renderBoundingBox && boundingBoxDims ? (
				<Box
					onPointerOver={() => {
						setShowText(true);
					}}
					onPointerLeave={() => {
						setShowText(false);
					}}
					key={robotId}
					args={[boundingBoxDims.x, boundingBoxDims.y, boundingBoxDims.z]}
					material={new MeshBasicMaterial({color: checkCollisions(robotId) ? 0xff0000 : 0x00ff00, wireframe: true})}
					position={new Vector3(0, 0, 0)}
				/>
			) : null}
		</>
	);
}

useGLTF.preload('./crazyflie.glb');
