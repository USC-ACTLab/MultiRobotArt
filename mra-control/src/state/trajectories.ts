import * as THREE from 'three';

export abstract class Trajectory {
  duration: number;
  constructor(duration: number){
    this.duration = duration;
  }

  abstract evaluate(t: number): THREE.Vector3;
}

export class PolynomialTrajectory extends Trajectory {
  coefs: THREE.Vector3[];
  constructor(duration: number, coeficients: THREE.Vector3[]){
    super(duration);
    this.coefs = coeficients;
  }

  evaluate(t: number): THREE.Vector3 { 
    const newPos = new THREE.Vector3();
    this.coefs.map((coefficient, i) => {
      newPos.addScaledVector(coefficient, Math.pow(t, i));
    });
    return newPos;
  }
}

