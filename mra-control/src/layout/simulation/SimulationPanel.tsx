import {Canvas} from '@react-three/fiber';

import {Simulation} from './Simulation';
import React from 'react';

export const SimulationPanel = () => {
	return (
		<div className="h-full w-full">
			<Canvas>
				<Simulation />
			</Canvas>
		</div>
	);
};
