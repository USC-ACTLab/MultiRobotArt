import { Modal, Tabs } from 'flowbite-react';
import {CancelButton} from '../../components/buttons/CancelButton';
import {useRobartState} from '../../state/useRobartState';
import {useUIState} from '../../state/useUIState';
import { useEffect, useState } from 'react';
import { IconButton } from '../buttons/IconButton';
import { faSync, faTrashCan } from '@fortawesome/free-solid-svg-icons';

export const RemoveGroupModal = () => {
	const RGModalOpen = useUIState((state) => state.RGModalOpen);
	const toggleRGModal = useUIState((state) => state.toggleRGModal);
    const groups = useRobartState((state) => state.timelineState.groups);
  // Add a new state variable for groupsToRemove
  const [groupsToRemove, setGroupsToRemove] = useState<string[]>([]);

  const robartState = useRobartState();
  // Function to handle checkbox change
  const handleCheckboxChange = (event:  React.ChangeEvent<HTMLInputElement>) => {
    if (event.target.checked) {
      setGroupsToRemove([...groupsToRemove, event.target.name]);
    } else {
      setGroupsToRemove(groupsToRemove.filter(group => group !== event.target.name));
    }
  };

	return (
		<>
			<Modal show={RGModalOpen} onClose={() => toggleRGModal()}>
				<Modal.Header>Remove Groups</Modal.Header>
				<Modal.Body>
				 <div>
					{Object.keys(groups).map((groupKey) => (
					<div key={groupKey}>
						<label>
				 			 <input type="checkbox" name={groupKey} onChange={handleCheckboxChange} />
				  			<span style={{ marginLeft: '10px' }}>{groupKey}</span>
						</label>
			  	</div>
			))}
		  </div>

				</Modal.Body>
				<Modal.Footer>
					<CancelButton onClick={toggleRGModal} />
					<IconButton color="warning" text="Remove" icon={faTrashCan} onClick={() => {
						console.log(groupsToRemove)
					robartState.removeGroups(groupsToRemove)}} />

                    </Modal.Footer>
                    </Modal>
                    </>)};