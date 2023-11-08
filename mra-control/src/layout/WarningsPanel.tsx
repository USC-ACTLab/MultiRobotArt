import React from 'react';
import CodeMirror from '@uiw/react-codemirror';
import {useRobartState} from '../state/useRobartState';

export const WarningsPanel = () => {
	const warnings = useRobartState.getState().getWarnings();
	return (
		<div className="overflow-auto h-full w-full ">
			<CodeMirror value={warnings} className="h-full w-full" readOnly={true} />
		</div>
	);
};
