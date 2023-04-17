import { faPlus } from '@fortawesome/free-solid-svg-icons';
import { Canvas } from '@react-three/fiber';
import { Button, Modal } from 'flowbite-react';
import { useEffect, useState } from 'react';
import * as THREE from 'three';

import { IconButton } from '../../components/buttons/IconButton';
import { VectorEditor } from '../../components/vector/VectorEditor';
import { useCurveEditorState } from '../../state/useCurveEditorState';
import { useRobartState } from '../../state/useRobartState';
import { useUIState } from '../../state/useUIState';
import { CurveEditor } from './CurveEditor';

export const CurveEditorModal = () => {
  const [points, setPoints] = useState<THREE.Vector3[]>([]);
  const curveEditorOpen = useUIState((state) => state.curveEditorOpen);
  const toggleCurveEditor = useUIState((state) => state.toggleCurveEditor);
  const bezierPoints = useCurveEditorState((state) => state.bezierControlPoints);
  const addPoint = useCurveEditorState((state) => state.addBezierControlPoint);
  const selectedPoint = useCurveEditorState((state) => state.selectedControlPoint);
  const pointCount = useCurveEditorState((state) => state.bezierControlPoints.length);

  useEffect(() => {
    setPoints(bezierPoints);
  }, [selectedPoint, pointCount]);

  return (
    <Modal show={curveEditorOpen} onClose={toggleCurveEditor} className="!w-full" size="w-full h-5/6">
      <Modal.Header>Curve Editor</Modal.Header>
      <Modal.Body className="h-[80vh]">
        <div className="flex h-full flex-row">
          <div className="w-1/4">
            <IconButton icon={faPlus} text="Add Point" onClick={() => addPoint(new THREE.Vector3(1, 1, 1))} />
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
