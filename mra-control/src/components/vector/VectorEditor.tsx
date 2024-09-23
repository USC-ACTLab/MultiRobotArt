import { VectorEditorItem } from './VectorEditorItem';
import React from 'react';
import THREE from 'three';
export const VectorEditor = ({ points }: { points: THREE.Vector3[] }) => {
	return (
		<div>
			{points.map((point, idx) => (
				<VectorEditorItem key={idx} id={idx} point={point} />
			))}
		</div>
	);
};
