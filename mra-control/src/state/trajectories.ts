import * as THREE from 'three';
import Sandbox from '@nyariv/sandboxjs';

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

	// eslint-disable-next-line @typescript-eslint/no-unused-vars
	evaluate(_t: number): THREE.Vector3 {
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
	startAngle: number;
	endAngle: number;

	constructor(duration: number, init_pos: THREE.Vector3, radius: number, axes = ['Y', 'Z'], radians = 2 * Math.PI, clockwise = true, startAngle = 0, endAngle = 360) {
		super(duration);
		this.radius = radius;
		this.axes = axes;
		this.radians = radians;
		this.clockwise = clockwise;
		this.pos = init_pos;
		this.startAngle = startAngle;
		this.endAngle = endAngle;
	}

	evaluate(t: number): THREE.Vector3 {
		// As t varies from 0 to 1, complete the desired arclength rotation for a given radius on a given axes
		// subtract 1 from v1 to start at 0, 0
		const v1 = this.radius * (Math.cos(t * (this.radians)) - 1); // TODO add starting and ending angles.
		const v2 = this.radius * Math.sin(t * (this.radians));
		var x = this.pos.x, y = this.pos.y, z = this.pos.z;
		if (this.axes.some((X) => X === 'X')) {
			x = v1 + this.pos.x;
		} else {
			y = v1 + this.pos.y;
		}

		if (this.axes.some((Z) => Z === 'Z')) {
			z = v2 + this.pos.z;
			console.warn('Z!');
		} else {
			console.warn(this.axes);
			y = v2 + this.pos.y;
		}

		return new THREE.Vector3(x, y, z);
	}
}

export class ComponentTrajectory extends Trajectory {
	trajX: Trajectory;
	trajY: Trajectory;
	trajZ: Trajectory;

	durX: number;
	durY: number;
	durZ: number;

	durScalingX: number;
	durScalingY: number;
	durScalingZ: number;
	constructor(duration: number, durX: number, trajX: Trajectory, durY: number, trajY: Trajectory, durZ: number, trajZ: Trajectory) {
		super(duration);
		this.trajX = trajX;
		this.trajY = trajY;
		this.trajZ = trajZ;

		this.durX = durX;
		this.durY = durY;
		this.durZ = durZ;

		// Rescale the duration for x, y, z so that it can end midway through the whole trajectory
		// e.g. if two different goto commands are specified for y and z axes 
		this.durScalingX = durX / duration;
		this.durScalingY = durY / duration;
		this.durScalingZ = durZ / duration;
		console.warn(this.durScalingX, this.durScalingY, this.durScalingZ);
	}
	
	evaluate(t: number): THREE.Vector3 {
		var x = 0, y = 0, z = 0;
		if (t / this.durScalingX > 1) {
			x = this.trajX.evaluate(1).x;
		} else {
			x = this.trajX.evaluate(t / this.durScalingX).x;
		}

		if (t / this.durScalingY > 1) {
			y = this.trajY.evaluate(1).y;
		} else {
			y = this.trajY.evaluate(t / this.durScalingY).y;
		}

		if (t * this.durScalingZ > 1) {
			z = this.trajZ.evaluate(1).z;
		} else {
			z = this.trajZ.evaluate(t / this.durScalingZ).z;
		}

		return new THREE.Vector3(x, y, z);
	}
}

export class ParametricTrajectory extends Trajectory {
	duration: number;
	startTime: number;
	endTime: number;
	timeScaling: number;
	xFunction: (t: number) => number;
	yFunction: (t: number) => number;
	zFunction: (t: number) => number;
	yawFunction: (t: number) => number;
	initPos: THREE.Vector3;
	offset: THREE.Vector3;

	constructor(initPos: THREE.Vector3, x: string, y: string, z: string, yaw: string, startTime: number, endTime: number, timeScaling: number) {
		const duration = (endTime - startTime) * timeScaling;
		super(duration);
		this.initPos = initPos;
		this.duration = duration;
		this.startTime = startTime;
		this.endTime = endTime;
		this.timeScaling = timeScaling;
		this.xFunction = this.strToFunction(x);
		this.yFunction = this.strToFunction(y);
		this.zFunction = this.strToFunction(z);
		this.yawFunction = this.strToFunction(yaw);

		// Make sure the function always starts at the correct position.
		this.offset = initPos.sub(new THREE.Vector3(this.xFunction(startTime), this.yFunction(startTime), this.zFunction(startTime)));
	}
	
	strToFunction(functionPlainText: string): (t: number) => number {
		const functionLambda = ((t: number) => {
			const regexMatch = /(?<![a-zA-Z])t(?![a-zA-Z])/g;
			//TODO append imports like cos, sin, tan, ...etc.
			// eslint-disable-next-line @typescript-eslint/naming-convention
			const plainTextWithTInserted = functionPlainText.replaceAll(regexMatch, String(t));
			const sandbox = new Sandbox();
			const exec = sandbox.compile(plainTextWithTInserted);
			const result = exec().run();
			// TODO Sanity checks, or at least fail gracefully...
			return result as number;
		});
		return functionLambda;
	}

	evaluate(t: number): THREE.Vector3 {
		const x = this.xFunction(t);
		const y = this.yFunction(t);
		const z = this.zFunction(t);
		return new THREE.Vector3(x, y, z);
	}
}

export class NegateTrajectory extends Trajectory {
	duration: number;
	initPos: THREE.Vector3;
	originalTrajectory: Trajectory;
	constructor(initPos: THREE.Vector3, originalTrajectory: Trajectory) {
		const duration = originalTrajectory.duration;
		super(duration);
		this.duration = duration;
		this.initPos = new THREE.Vector3;
		this.initPos.copy(initPos);
		this.originalTrajectory = originalTrajectory;
	}

	evaluate(t: number): THREE.Vector3 {
		const originalTrajectoryPosition = this.originalTrajectory.evaluate(t);
		const initPos = new THREE.Vector3().copy(this.initPos);
		const desiredPosition = initPos.multiplyScalar(2).sub(originalTrajectoryPosition);
		console.warn(desiredPosition, this.initPos, t);
		return desiredPosition;
	}
}