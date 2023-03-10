import { OrbitControls, Sky, Stage } from "@react-three/drei";
import {
  CatmullRomLine,
  Circle,
  Environment,
  GizmoHelper,
  GizmoViewport,
  Grid,
  Point,
  PointMaterial,
  Points,
  Sphere,
} from "@react-three/drei/core";
import { useCurveEditorState } from "../../state/useCurveEditorState";
import { computeBezierPoint } from "../../tools/vectors/bezier";
import { useFrame } from "@react-three/fiber";
import { useRef } from "react";
import { type Group, type Mesh } from "three";
import { Crazyflie } from "../../components/vector/Crazyflie";

export const CurveEditor = () => {
  const bezierControlPoints = useCurveEditorState(
    (state) => state.bezierControlPoints
  );
  const bezierLinePoints = useCurveEditorState(
    (state) => state.bezierLinePoints
  );

  const duration = bezierControlPoints.length;
  const marker = useRef<any>();

  useFrame(({ clock }) => {
    const a = clock.getElapsedTime();
    const nextPoint = computeBezierPoint(
      bezierControlPoints,
      (a % duration) / duration
    );
    marker.current?.position.set(nextPoint.x, nextPoint.y, nextPoint.z);
  });

  return (
    <>
      <OrbitControls
        makeDefault
        maxPolarAngle={Math.PI * (1 / 2 - 1 / 10)}
        minPolarAngle={0}
      />
      <Environment preset="warehouse" />
      <GizmoHelper alignment="bottom-right" margin={[80, 80]}>
        <GizmoViewport
          axisColors={["#9d4b4b", "#2f7f4f", "#3b5b9d"]}
          labelColor="white"
        />
      </GizmoHelper>
      <Grid
        position={[0, -0.001, 0]}
        args={[10.5, 10.5]}
        cellSize={0.6}
        cellThickness={1}
        cellColor={"black"}
        sectionSize={3.3}
        sectionThickness={1.5}
        sectionColor={"black"}
        fadeDistance={100}
        fadeStrength={0.5}
        followCamera={true}
        infiniteGrid={true}
      />
      <OrbitControls makeDefault />
      <ambientLight intensity={0.1} />
      <directionalLight color="blue" position={[0, 5, 5]} />
      <Sky
        distance={450000}
        sunPosition={[0, 1, 0]}
        inclination={0}
        azimuth={0.25}
      />
      <group>
        {bezierControlPoints.map((point, i) =>
          i === 0 || i === bezierControlPoints.length - 1 ? (
            <Sphere key={i} position={point} scale={0.2}>
              <PointMaterial
                transparent
                vertexColors
                size={25}
                sizeAttenuation={false}
                depthWrite={false}
              />{" "}
            </Sphere>
          ) : (
            <Sphere key={i} position={point} scale={0.2}>
              <PointMaterial
                transparent
                vertexColors
                size={25}
                sizeAttenuation={false}
                depthWrite={false}
              />
            </Sphere>
          )
        )}

        <group ref={marker}>
          <Crazyflie />
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
