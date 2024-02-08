import { RemoveGroupModal } from '@MRAControl/components/modal/RemoveGroup';
import {useRobartState} from '@MRAControl/state/useRobartState';
import { useUIState } from '@MRAControl/state/useUIState';
import {faPlusCircle, faTrashCan} from '@fortawesome/free-solid-svg-icons';
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

const removeGroup = (() => {
	const groups = useRobartState.getState().timelineState.groups;
	const numGroups = Object.keys(groups).length; 
	const groupId = 'group' + (numGroups -1);
	useRobartState.getState().removeGroup(groupId);
});

export const RemoveTimelineGroupLabel = () => {
	const toggleRGModal = useUIState.getState().toggleRGModal;
	return (
		<>
		<div className='h-16 w-16'><Button onClick={() => {toggleRGModal();}}  style={{width: 'fit-content'}}>
			Remove Group <FontAwesomeIcon icon={faTrashCan} />
		</Button></div>
		<RemoveGroupModal />
		</>
	);
};

