import { OrbitControls, Sky, Stage } from "@react-three/drei";
import {
  Environment,
  GizmoHelper,
  GizmoViewport,
  Grid,
  Sphere,
} from "@react-three/drei/core";
import { useCurveEditorState } from "../../state/useCurveEditorState";

export const CurveEditor = () => {
  const bezierPoints = useCurveEditorState((state) => state.bezierPoints);
  return (
    <>
      <OrbitControls makeDefault />
      <Environment preset="city" />
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
      <directionalLight color="red" position={[0, 0, 5]} />
      <Sky
        distance={450000}
        sunPosition={[0, 1, 0]}
        inclination={0}
        azimuth={0.25}
      />
      <group>
        {bezierPoints.map((point, i) => (
          <mesh key={i}>
            <Sphere position={point} scale={0.2} />
          </mesh>
        ))}
      </group>
    </>
  );
};
