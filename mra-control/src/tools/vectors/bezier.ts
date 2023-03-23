import _ from 'lodash';
import * as THREE from 'three';

const BINOMIAL_COEFFICIENTS = [
  [1],
  [1, 1],
  [1, 2, 1],
  [1, 3, 3, 1],
  [1, 4, 6, 4, 1],
  [1, 5, 10, 10, 5, 1],
  [1, 6, 15, 20, 15, 6, 1],
  [1, 7, 21, 35, 35, 21, 7, 1],
  [1, 8, 28, 56, 70, 56, 28, 8, 1],
  [1, 9, 36, 84, 126, 126, 84, 36, 9, 1],
  [1, 10, 45, 120, 210, 252, 210, 120, 45, 10, 1],
];

export const MAX_CONTROL_POINTS = BINOMIAL_COEFFICIENTS.length;

export const computeBezierPoint = (controlPoints: THREE.Vector3[], t: number): THREE.Vector3 => {
  const b = new THREE.Vector3(0, 0, 0);
  const n = controlPoints.length - 1;
  for (let i = 0; i <= n; i++) {
    b.addScaledVector(controlPoints[i], BINOMIAL_COEFFICIENTS[n][i] * Math.pow(1 - t, n - i) * Math.pow(t, i));
  }
  return b;
};

export const interpolateBezierPoints = (controlPoints: THREE.Vector3[], interpolationCount: number): THREE.Vector3[] => {
  return Array(interpolationCount)
    .fill(0)
    .map((_, i) => computeBezierPoint(controlPoints, i / interpolationCount));
};
