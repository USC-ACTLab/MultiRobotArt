import { VectorEditorItem } from './VectorEditorItem';

export const VectorEditor = ({ points }: { points: THREE.Vector3[] }) => {
  return (
    <div>
      {points.map((point, idx) => (
        <VectorEditorItem key={idx} id={idx} point={point} />
      ))}
    </div>
  );
};
