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
  constructor(duration: number, coefficients: THREE.Vector3[]){
    super(duration);
    this.coefs = coefficients;
  }

  evaluate(t: number): THREE.Vector3 { 
    const newPos = new THREE.Vector3();
    this.coefs.map((coefficient, i) => {
      newPos.addScaledVector(coefficient, Math.pow(t, i));
    });
    return newPos;
  }
}

export class CircleTrajectory extends Trajectory{
    radius: number;
    axes: String[];
    radians: number;
    clockwise: boolean;
    pos: THREE.Vector3;
    constructor(duration: number, init_pos: THREE.Vector3, radius: number, axes=['Y', 'Z'], radians=2*Math.PI, clockwise=false){
        super(duration);
        this.radius = radius;
        this.axes = axes;
        this.radians = radians;
        this.clockwise = clockwise;
        this.pos = init_pos
    }
    evaluate(t: number): THREE.Vector3{
        // As t varies from 0 to 1, complete the desired arclength rotation for a given radius on a given axes
        // subtract 1 from v1 to start at 0, 0
        const v1 = this.radius * Math.cos(t*Math.PI / (this.radians / 2*Math.PI)) - 1;
        const v2 = this.radius * Math.sin(t*Math.PI / (this.radians / 2*Math.PI));
        var x = 0, y = 0, z = 0;
        if(this.axes.some(x => x === 'X')){
            x = v1 + this.pos.x;
        }
        else{
            x = 0
            y = v1 + this.pos.y;
        }
        if(this.axes.some(x => x === 'Z')){
            z = v2 + this.pos.z;
        }
        else{
            y = v2 + this.pos.y;
            z = 0;
        }
        console.log(this.radians)
        return new THREE.Vector3(x, y, z);
    }
}