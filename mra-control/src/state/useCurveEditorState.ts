import { create } from "zustand";
import * as THREE from "three";
import { immer } from "zustand/middleware/immer";
import { subscribeWithSelector } from "zustand/middleware";
import { interpolateBezierPoints } from "../tools/vectors/bezier";

export interface CurveEditorState {
  bezierControlPoints: THREE.Vector3[];
  selectedPoint: number | undefined;
  bezierLinePoints: THREE.Vector3[];
}

export interface CurveEditorActions {
  addBezierControlPoint: (point: THREE.Vector3) => void;
  removeBezierControlPoint: (idx: number) => void;
  updateBezierControlPoint: (idx: number, point: THREE.Vector3) => void;
  setSelectedControlPoint: (idx: number | undefined) => void;
  updateBezierInterpolationPoints: () => void;
}

export type CurveEditorStoreState = CurveEditorState & CurveEditorActions;

const INTERPOLATION_COUNT = 1000;

/**
 * Store for aspects of the curve editor.
 */
export const useCurveEditorState = create<CurveEditorStoreState>()(
  subscribeWithSelector(
    immer((set, get) => ({
      bezierControlPoints: [
        new THREE.Vector3(1, 1, 1),
        new THREE.Vector3(2, 5, 4),
      ],
      selectedPoint: undefined,
      bezierLinePoints: [],
      addBezierControlPoint: (point) => {
        const points = [...get().bezierControlPoints];
        points.push(point);
        set({
          bezierControlPoints: points,
        });
      },
      removeBezierControlPoint: (idx) => {
        const points = [...get().bezierControlPoints];
        points.splice(idx, 1);
        console.log(points);
        set({
          bezierControlPoints: points,
        });
      },
      updateBezierControlPoint: (idx, point) =>
        set((state) => {
          state.bezierControlPoints[idx] = point;
        }),
      setSelectedControlPoint: (idx) => {
        set({
          selectedPoint: idx,
        });
      },
      updateBezierInterpolationPoints: () => {
        const interpolationPoints = interpolateBezierPoints(
          get().bezierControlPoints,
          INTERPOLATION_COUNT
        );
        set({ bezierLinePoints: interpolationPoints });
      },
    }))
  )
);

useCurveEditorState.subscribe(
  (state) => state.bezierControlPoints,
  () => {
    useCurveEditorState.getState().updateBezierInterpolationPoints();
  }
);
