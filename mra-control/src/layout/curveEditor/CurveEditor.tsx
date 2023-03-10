import { OrbitControls, Sky, Stage } from "@react-three/drei";
import {
  CatmullRomLine,
  Circle,
  Environment,
  GizmoHelper,
  GizmoViewport,
  Grid,
  Sphere,
} from "@react-three/drei/core";
import { MeshPhongMaterial } from "three";
import { useCurveEditorState } from "../../state/useCurveEditorState";

export const CurveEditor = () => {
  const bezierControlPoints = useCurveEditorState(
    (state) => state.bezierControlPoints
  );
  const bezierLinePoints = useCurveEditorState(
    (state) => state.bezierLinePoints
  );
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
        cellColor={"#6f6f6f"}
        sectionSize={3.3}
        sectionThickness={1.5}
        sectionColor={"#9d4b4b"}
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
        {bezierControlPoints.map((point, i) => (
          <Sphere key={i} position={point} scale={0.2}>
            <meshLambertMaterial color="blue" />
          </Sphere>
        ))}
        <CatmullRomLine
          points={bezierLinePoints}
          closed={false}
          curveType="centripetal"
          tension={0.5}
          color="black"
          lineWidth={2}
          dashed={true}
        />
      </group>
    </>
  );
};
