import { Button, Modal } from "flowbite-react";
import { useRobartState } from "../state/useRobartState";
import { useUIState } from "../state/useUIState";
import { Canvas } from "@react-three/fiber";
import { CurveEditor } from "./curveEditor/CurveEditor";
import { useMemo } from "react";
import { useCurveEditorState } from "../state/useCurveEditorState";
import * as THREE from "three";

export const CurveEditorModal = () => {
  const curveEditorOpen = useUIState((state) => state.curveEditorOpen);
  const toggleCurveEditor = useUIState((state) => state.toggleCurveEditor);
  const bezierPoints = useCurveEditorState((state) => state.bezierPoints);
  const addPoint = useCurveEditorState((state) => state.addBezierPoint);
  return (
    <Modal
      show={curveEditorOpen}
      onClose={toggleCurveEditor}
      className="!w-full"
      size="7xl"
    >
      <Modal.Header>Curve Editor</Modal.Header>
      <Modal.Body className="h-[64rem]">
        <div className="flex h-full flex-row">
          <div className="w-80">
            <Button onClick={() => addPoint(new THREE.Vector3(1, 1, 1))}>
              Add Point
            </Button>
            <div className="flex flex-col gap-2"></div>
          </div>
          <div className="flex h-full flex-grow">
            {useMemo(
              () => (
                <Canvas>
                  <CurveEditor />
                </Canvas>
              ),
              []
            )}
          </div>
        </div>
      </Modal.Body>
      <Modal.Footer></Modal.Footer>
    </Modal>
  );
};
