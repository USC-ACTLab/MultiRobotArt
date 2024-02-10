import React, { useEffect, useState } from 'react';
import CodeMirror from '@uiw/react-codemirror';
import {useRobartState} from '../state/useRobartState';

export const WarningsPanel = () => {
    const [key, setKey] = useState(0);
	const [warnings, setWarnings] = useState('')

    useEffect(() => {
        const newWarnings = useRobartState.getState().getWarnings();
		setWarnings(newWarnings);
    }, [key]);
	

	useEffect(() => {
        setKey(prevKey => prevKey + 1);
    }, []);
	return (
		<div className="overflow-auto h-full w-full " key={key}>
			<CodeMirror value={warnings} className="h-full w-full" readOnly={true} />
		</div>
	);
};
