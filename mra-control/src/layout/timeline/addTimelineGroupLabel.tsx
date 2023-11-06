import {useRobartState} from '@MRAControl/state/useRobartState';
import {faPlusCircle} from '@fortawesome/free-solid-svg-icons';
import {FontAwesomeIcon} from '@fortawesome/react-fontawesome';
import {Button} from 'flowbite-react';
import React from 'react';

const addNewGroup = (() => {
	const groups = useRobartState.getState().timelineState.groups;
	const numGroups = Object.keys(groups).length; 
	const groupName = 'group ' + numGroups;
	useRobartState.getState().addGroup(groupName);
});

export const AddTimelineGroupLabel = () => {
	return (
		<div className='h-16 w-16'><Button onClick={addNewGroup} style={{width: 'fit-content'}}>
			Add Group <FontAwesomeIcon icon={faPlusCircle} />
		</Button></div>
	);
};

