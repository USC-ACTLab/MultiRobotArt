/* eslint-disable import/no-extraneous-dependencies */
// @ts-nocheck
import {a, useSpring} from '@react-spring/three';
import {Sphere} from '@react-three/drei';
import {ThreeEvent, useThree} from '@react-three/fiber';
import {useGesture} from '@use-gesture/react';
import {useState} from 'react';
import * as THREE from 'three';
import React from 'react';

import {useCurveEditorState} from '../../state/useCurveEditorState';
import { IntersectionEvent } from '@react-three/fiber/dist/declarations/src/core/events';

type BezierControlPointProps = {
	id: number;
	point: THREE.Vector3;
};

export const BezierControlPoint = ({id, point}: BezierControlPointProps) => {
	const [dragBase, setDragBase] = useState<THREE.Vector3 | undefined>(undefined);
	const selectedPoint = useCurveEditorState((state) => state.selectedControlPoint);

	const setSelectedControlPoint = useCurveEditorState((state) => state.setSelectedControlPoint);

	const updateBezierControlPoint = useCurveEditorState((state) => state.updateBezierControlPoint);

	const {size, viewport, camera} = useThree();


	const [spring, set] = useSpring(() => ({
		scale: new THREE.Vector3(1, 1, 1),
		position: new THREE.Vector3(point.x, point.y, point.z),
		config: {mass: 1, tension: 0, friction: 18},
	}));

	const bind = useGesture({
		onDrag: ({xy: [mouseX, mouseY]}) => {
			if (!dragBase) return;
			const mouseRay = new THREE.Raycaster();
			mouseRay.setFromCamera(
				{
					x: ((mouseX - viewport.left) / size.width) * 2 - 1,
					y: -((mouseY - viewport.top) / size.height) * 2 + 1,
				},
				camera,
			);

			// create pgroup parallel to camera that passes through dragBase
			const plane = new THREE.Plane();
			plane.setFromNormalAndCoplanarPoint(camera.getWorldDirection(new THREE.Vector3(0, 0, 1)), dragBase);

			//   compute intersection of mouse ray and pgroup
			const intersection = new THREE.Vector3();
			mouseRay.ray.intersectPlane(plane, intersection);

			set({
				position: intersection
			});
			updateBezierControlPoint(id, intersection.clone());
		},
		onHover: ({hovering}) => {
			set({scale: hovering ? new THREE.Vector3(1.2, 1.2, 1.2) : new THREE.Vector3(1, 1, 1)});
		},
		onDragStart: () => {
			setSelectedControlPoint(id);
			setDragBase(new THREE.Vector3(spring.position));
		},
		onDragEnd: () => {
			setDragBase(undefined);
			setSelectedControlPoint(undefined);
		},
	});

	return (
		<a.mesh {...spring} {...bind()} castShadow>
			<Sphere scale={0.2}>
				<meshBasicMaterial color={selectedPoint === id ? '#FF00FF' : '#000000'} />
				<pointLight color={0xffffff} intensity={1} />
			</Sphere>
		</a.mesh>
	);
};

// <a.mesh {...spring} {...bind()}>

// </a.mesh>
