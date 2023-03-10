import { Button, Modal } from "flowbite-react";
import { useRobartState } from "../state/useRobartState";
import { useUIState } from "../state/useUIState";
import { Canvas } from "@react-three/fiber";
import { CurveEditor } from "./curveEditor/CurveEditor";
import { useCurveEditorState } from "../state/useCurveEditorState";
import * as THREE from "three";
import { IconButton } from "../components/buttons/IconButton";
import { faPlus } from "@fortawesome/free-solid-svg-icons";
import { VectorEditor } from "../components/vector/VectorEditor";
import { useEffect, useState } from "react";

export const CurveEditorModal = () => {
  const [points, setPoints] = useState<THREE.Vector3[]>([]);
  const curveEditorOpen = useUIState((state) => state.curveEditorOpen);
  const toggleCurveEditor = useUIState((state) => state.toggleCurveEditor);
  const bezierPoints = useCurveEditorState(
    (state) => state.bezierControlPoints
  );
  const addPoint = useCurveEditorState((state) => state.addBezierControlPoint);
  const selectedPoint = useCurveEditorState((state) => state.selectedPoint);
  const pointCount = useCurveEditorState(
    (state) => state.bezierControlPoints.length
  );

  useEffect(() => {
    console.log("SETTING");
    setPoints(bezierPoints);
  }, [selectedPoint, pointCount]);

  return (
    <Modal
      show={curveEditorOpen}
      onClose={toggleCurveEditor}
      className="!w-full"
      size="w-full"
    >
      <Modal.Header>Curve Editor</Modal.Header>
      <Modal.Body className="h-[64rem]">
        <div className="flex h-full flex-row">
          <div className="w-1/4">
            <IconButton
              icon={faPlus}
              text="Add Point"
              onClick={() => addPoint(new THREE.Vector3(1, 1, 1))}
            />
            <VectorEditor points={points} />
            <div className="flex flex-col gap-2"></div>
          </div>
          <div className="flex h-full flex-grow">
            <Canvas>
              <CurveEditor />
            </Canvas>
          </div>
        </div>
      </Modal.Body>
      <Modal.Footer></Modal.Footer>
    </Modal>
  );
};
