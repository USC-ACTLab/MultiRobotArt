import { VectorEditorItem } from "./VectorEditorItem";

export const VectorEditor = ({ points }: { points: THREE.Vector3[] }) => {
  console.log("Vector Editor Rerender!", points);
  return (
    <div>
      {points.map((point, idx) => (
        <VectorEditorItem key={idx} id={idx} point={point} />
      ))}
    </div>
  );
};
