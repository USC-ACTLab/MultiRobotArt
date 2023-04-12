import { faTrashCan } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { Button, TextInput } from 'flowbite-react';
import { useEffect, useState } from 'react';
import * as THREE from 'three';

import { useCurveEditorState } from '../../state/useCurveEditorState';
import { IconButton } from '../buttons/IconButton';

export const VectorEditorItem = ({ id, point }: { id: number; point: THREE.Vector3 }) => {
  const updatePoint = useCurveEditorState((state) => state.updateBezierControlPoint);
  const removeBezierPoint = useCurveEditorState((state) => state.removeBezierControlPoint);
  const [pos, setPos] = useState({
    x: `${point.x}`,
    y: `${point.y}`,
    z: `${point.z}`,
  });

  useEffect(() => {
    updatePoint(id, new THREE.Vector3(parseFloat(pos.x) || 0, parseFloat(pos.y) || 0, parseFloat(pos.z) || 0));
  }, [pos]);

  useEffect(() => {
    setPos({ x: `${point.x}`, y: `${point.y}`, z: `${point.z}` });
  }, [point]);

  return (
    <div className="flex items-center gap-2 p-2">
      <span>{`${id + 1}.`}</span>
      <TextInput
        className="text-center"
        value={pos.x}
        onChange={(e) =>
          setPos((oldState) => {
            return { ...oldState, x: e.target.value };
          })
        }
      />
      <TextInput
        className="text-center"
        value={pos.y}
        onChange={(e) =>
          setPos((oldState) => {
            return { ...oldState, y: e.target.value };
          })
        }
      />
      <TextInput
        className="text-center"
        value={pos.z}
        onChange={(e) =>
          setPos((oldState) => {
            return { ...oldState, z: e.target.value };
          })
        }
      />
      <button onClick={() => removeBezierPoint(id)}>
        <FontAwesomeIcon className="text-blue-600" icon={faTrashCan} />
      </button>
    </div>
  );
};
