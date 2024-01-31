import {Canvas} from '@react-three/fiber';

import {Simulation} from './Simulation';
import React, { useState } from 'react';
import { PerformanceMonitor } from '@react-three/drei';



export const SimulationPanel = () => {
	const [dpr, setDpr] = useState(1.5)

	const onPerformanceDecline = (() => {
		const newDpr = Math.max(dpr - 0.5, 0.5);
		setDpr(newDpr);
	});

	const onPerformanceIncline = (() => {
		const newDpr = Math.min(dpr + 0.5, 2.0);
		setDpr(newDpr);
	});

	return (
		<div className="h-full w-full">
			<Canvas dpr={dpr}>
				<Simulation />
				<PerformanceMonitor onIncline={onPerformanceIncline} onDecline={onPerformanceDecline}></PerformanceMonitor>
			</Canvas>
		</div>
	);
};
