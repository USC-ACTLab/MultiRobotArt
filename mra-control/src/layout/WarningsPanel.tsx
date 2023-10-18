import React from 'react';

import {useRobartState} from '../state/useRobartState';

export const WarningsPanel = () => {
	const warnings = useRobartState.getState().getWarnings();
	return (
		<div className="h-full w-full">
			{warnings}
		</div>
	);
};
