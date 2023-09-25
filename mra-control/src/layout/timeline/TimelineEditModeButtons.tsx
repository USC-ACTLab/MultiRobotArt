import {useRobartState} from '@MRAControl/state/useRobartState';
import {faArrowsLeftRight, faEraser, faPlusCircle} from '@fortawesome/free-solid-svg-icons';
import {FontAwesomeIcon} from '@fortawesome/react-fontawesome';
import {Button} from 'flowbite-react';

export const TimelineEditModeButtons = () => {
	const timelineMode = useRobartState((state) => state.timelineState.mode);
	const setTimelineMode = useRobartState((state) => state.setTimelineMode);

	return (
		<div className="flex gap-3 pt-2 pl-3">
			<Button.Group>
				<Button color={timelineMode === 'ADD' ? 'info' : 'gray'} onClick={() => {
					setTimelineMode('ADD'); 
				}}>
					<FontAwesomeIcon icon={faPlusCircle} />
				</Button>
				<Button color={timelineMode === 'MOVE' ? 'info' : 'gray'} onClick={() => {
					setTimelineMode('MOVE'); 
				}}>
					<FontAwesomeIcon icon={faArrowsLeftRight} />
				</Button>
				<Button color={timelineMode === 'ERASE' ? 'info' : 'gray'} onClick={() => {
					setTimelineMode('ERASE'); 
				}}>
					<FontAwesomeIcon icon={faEraser} />
				</Button>
			</Button.Group>
		</div>
	);
};
