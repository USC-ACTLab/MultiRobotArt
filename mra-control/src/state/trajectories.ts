import * as THREE from 'three';

export abstract class Trajectory {
	constructor(public duration: number) {
		this.duration = duration;
	}

	abstract evaluate(t: number): THREE.Vector3;
}

// export class TrajectoryWrapper extends Trajectory {
// 	traj: Trajectory | undefined;
// 	trajectoryType: new (duration: number) => Trajectory;
// 	args: any[];
// 	constructor(trajectoryType: new (arg: any[]) => Trajectory, ...args: any[]) {
// 		super(-1.0);
// 		this.traj = undefined;
// 		this.trajectoryType = trajectoryType;
// 		this.args = args;
// 	}

// 	evaluate(t: number) {
// 		if (this.traj === undefined) {
// 			this.traj = new this.trajectoryType(this.args);
// 			this.duration = this.traj.duration;
// 		}

// 		return this.traj.evaluate(t);
// 	}
// }

export class NullTrajectory extends Trajectory {
	constructor() {
		super(0.1);
	}

	// eslint-disable-next-line @typescript-eslint/no-unused-vars
	evaluate(t: number): THREE.Vector3 {
		return new THREE.Vector3();
	}
}

export class Hover extends Trajectory {
	position: THREE.Vector3;
	constructor(position: THREE.Vector3) {
		super(0.1);
		this.position = position;
	}

	evaluate(t: number): THREE.Vector3 {
		return this.position;
	}
}

export class PolynomialTrajectory extends Trajectory {
	coefs: THREE.Vector3[];
	constructor(duration: number, coefficients: THREE.Vector3[]) {
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

export class CircleTrajectory extends Trajectory {
	radius: number;
	axes: string[];
	radians: number;
	clockwise: boolean;
	pos: THREE.Vector3;
	constructor(duration: number, init_pos: THREE.Vector3, radius: number, axes = ['Y', 'Z'], radians = 2 * Math.PI, clockwise = false) {
		super(duration);
		this.radius = radius;
		this.axes = axes;
		this.radians = radians;
		this.clockwise = clockwise;
		this.pos = init_pos;
	}

	evaluate(t: number): THREE.Vector3 {
		// As t varies from 0 to 1, complete the desired arclength rotation for a given radius on a given axes
		// subtract 1 from v1 to start at 0, 0
		const v1 = this.radius * Math.cos(t * (this.radians)) - 1;
		const v2 = this.radius * Math.sin(t * (this.radians));
		var x = this.pos.x, y = this.pos.y, z = this.pos.z;
		if (this.axes.some(x => x === 'X')) {
			x = v1 + this.pos.x;
		} else {
			y = v1 + this.pos.y;
		}

		if (this.axes.some(x => x === 'Z')) {
			z = v2 + this.pos.z;
		} else {
			y = v2 + this.pos.y;
		}

		return new THREE.Vector3(x, y, z);
	}
}

export class ComponentTrajectory extends Trajectory {
	trajX: Trajectory;
	trajY: Trajectory;
	trajZ: Trajectory;
	durScalingX: number;
	durScalingY: number;
	durScalingZ: number;
	constructor(duration: number, durX: number, trajX: Trajectory, durY: number, trajY: Trajectory, durZ: number, trajZ: Trajectory) {
		super(duration);
		this.trajX = trajX;
		this.trajY = trajY;
		this.trajZ = trajZ;
		this.durScalingX = durX / duration;
		this.durScalingY = durY / duration;
		this.durScalingZ = durZ / duration;
	}

	evaluate(t: number): THREE.Vector3 {
		var x = 0, y = 0, z = 0;
		if (t * this.durScalingX > 1) {
			x = this.trajX.evaluate(1).x;
		} else {
			x = this.trajX.evaluate(t * this.durScalingX).x;
		}

		if (t * this.durScalingY > 1) {
			y = this.trajY.evaluate(1).y;
		} else {
			y = this.trajY.evaluate(t * this.durScalingY).y;
		}

		if (t * this.durScalingZ > 1) {
			z = this.trajZ.evaluate(1).z;
		} else {
			z = this.trajZ.evaluate(t * this.durScalingZ).z;
		}

		return new THREE.Vector3(x, y, z);

	}
}