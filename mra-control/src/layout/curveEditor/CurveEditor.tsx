import { OrbitControls, Sky } from '@react-three/drei';
import { CatmullRomLine, Environment, GizmoHelper, GizmoViewport, Grid, Sphere } from '@react-three/drei/core';
import { useFrame } from '@react-three/fiber';
import { useEffect, useRef } from 'react';

import { Crazyflie } from '../../components/vector/Crazyflie';
import { useCurveEditorState } from '../../state/useCurveEditorState';
import { computeBezierPoint } from '../../tools/vectors/bezier';
import { BezierControlPoint } from './BezierControlPoint';

export const CurveEditor = () => {
  const bezierControlPoints = useCurveEditorState((state) => state.bezierControlPoints);
  const bezierLinePoints = useCurveEditorState((state) => state.bezierLinePoints);
  const selectedControlPoint = useCurveEditorState((state) => state.selectedControlPoint);

  const duration = bezierControlPoints.length;
  const marker = useRef<any>();

  useFrame(({ clock }) => {
    const a = clock.getElapsedTime();
    const nextPoint = computeBezierPoint(bezierControlPoints, (a % duration) / duration);
    marker.current?.position.set(nextPoint.x, nextPoint.y, nextPoint.z);
  });

  return (
    <>
      <color attach="background" args={['black']} />
      <OrbitControls
        maxPolarAngle={Math.PI * (1 / 2 - 1 / 10)}
        minPolarAngle={0}
        maxDistance={20}
        enableRotate={selectedControlPoint === undefined}
      />
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
      <directionalLight color="blue" position={[0, 5, 5]} />
      <Sky distance={450000} sunPosition={[0, 1, 0]} inclination={0} azimuth={0.25} />
      <group>
        {bezierControlPoints.map((point, i) => (
          <BezierControlPoint key={i} id={i} point={point} />
        ))}

        <group ref={marker}>
          <Crazyflie robotId="" renderBoundingBox={false} />
        </group>

        <CatmullRomLine
          points={bezierLinePoints}
          closed={false}
          curveType="centripetal"
          tension={0.5}
          color="black"
          lineWidth={3}
          dashed={true}
          dashSize={0.5}
          dashScale={5}
        />
      </group>
    </>
  );
};
