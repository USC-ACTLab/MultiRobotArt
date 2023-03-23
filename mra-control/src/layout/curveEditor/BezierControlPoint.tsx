import { a, useSpring } from '@react-spring/three';
import { Sphere } from '@react-three/drei';
import { useThree } from '@react-three/fiber';
import { useGesture } from '@use-gesture/react';
import { useState } from 'react';
import * as THREE from 'three';

import { useCurveEditorState } from '../../state/useCurveEditorState';

interface BezierControlPointProps {
  id: number;
  point: THREE.Vector3;
}

export const BezierControlPoint = ({ id, point }: BezierControlPointProps) => {
  const [dragBase, setDragBase] = useState<THREE.Vector3 | undefined>(undefined);
  const selectedPoint = useCurveEditorState((state) => state.selectedControlPoint);

  const setSelectedControlPoint = useCurveEditorState((state) => state.setSelectedControlPoint);

  const updateBezierControlPoint = useCurveEditorState((state) => state.updateBezierControlPoint);

  const { size, viewport, camera } = useThree();

  const aspect = viewport.aspect;

  const [spring, set] = useSpring(() => ({
    scale: [1, 1, 1],
    position: [point.x, point.y, point.z],
    config: { mass: 1, tension: 0, friction: 18 },
  }));

  const bind = useGesture({
    onDrag: ({ xy: [mouseX, mouseY] }) => {
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
        position: intersection.toArray(),
      });
      updateBezierControlPoint(id, intersection.clone());
    },
    onHover: ({ hovering }) => {
      set({ scale: hovering ? [1.2, 1.2, 1.2] : [1, 1, 1] });
    },
    onDragStart: () => {
      setSelectedControlPoint(id);
      const [x, y, z] = spring.position.get();
      setDragBase(new THREE.Vector3(x, y, z));
    },
    onDragEnd: () => {
      setDragBase(undefined);
      setSelectedControlPoint(undefined);
    },
  }) as any;

  return (
    <a.mesh {...spring} {...bind()} castShadow>
      <Sphere scale={0.2}>
        <meshBasicMaterial color={selectedPoint === id ? '#FF00FF' : '#000000'} />
      </Sphere>
    </a.mesh>
  );
};

// <a.mesh {...spring} {...bind()}>

// </a.mesh>
