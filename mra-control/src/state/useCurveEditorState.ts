import { create } from "zustand";
import * as THREE from "three";

export interface CurveEditorState {
  bezierPoints: THREE.Vector3[];
}

export interface CurveEditorActions {
  addBezierPoint: (point: THREE.Vector3) => void;
}

export type CurveEditorStoreState = CurveEditorState & CurveEditorActions;

/**
 * Store for aspects of the curve editor.
 */
export const useCurveEditorState = create<CurveEditorStoreState>()(
  (set, get) => ({
    bezierPoints: [new THREE.Vector3(1, 1, 1), new THREE.Vector3(2, 5, 4)],
    addBezierPoint: (point) => {
      const points = [...get().bezierPoints];
      points.push(point);
      set({
        bezierPoints: points,
      });
    },
  })
);
