import {TimelineMarker} from '@MRAControl/components/timeline/TimelineMarker';
import {faRobot} from '@fortawesome/free-solid-svg-icons';

import {IconButton} from '../../components/buttons/IconButton';
import {TimelineGroupBody} from '../../components/timeline/TimelineGroupBody';
import {TimelineGroupLabel} from '../../components/timeline/TimelineGroupLabel';
import {useRobartState} from '../../state/useRobartState';
import {useUIState} from '../../state/useUIState';
import {RobotManagerModal} from '../robotManager/RobotManagerModal';
import {TimelineEditModeButtons} from './TimelineEditModeButtons';
import {TimelineSimulationButtons} from './TimelineSimulationButtons';
import React, {useEffect} from 'react';
import {AddTimelineGroupLabel} from './addTimelineGroupLabel';


export const Timeline = () => {
	const timelineState = useRobartState((state) => state.timelineState);
	const groups = Object.values(timelineState.groups);
	const toggleRobotManagerModal = useUIState((state) => state.toggleRobotManager);


	return (
		<div className="flex h-full w-full flex-col gap-2 rounded bg-blue-100">
			<div className="flex">
				<TimelineEditModeButtons />
				<div className="flex flex-grow" />
				<div className="flex gap-3 pt-2 pr-3">
					<TimelineSimulationButtons />
					<IconButton icon={faRobot} onClick={toggleRobotManagerModal} text="Manage Robots" />
					<RobotManagerModal />
				</div>
			</div>

			<div className="overflow-y-auto">
				<div className="flex flex-shrink-0 gap-2">
					<div className="ml-2 flex h-full flex-col gap-2">
						{groups.map((group) => (
							<TimelineGroupLabel group={group} key={group.id} />
						))}
						<AddTimelineGroupLabel></AddTimelineGroupLabel>
					</div>
					<div className="relative flex h-full w-full flex-col gap-2 overflow-x-auto">
						{groups.map((group) => (
							<TimelineGroupBody group={group} key={group.id} />
						))}

						<TimelineMarker />
					</div>
				</div>
			</div>
		</div>
	);
};
