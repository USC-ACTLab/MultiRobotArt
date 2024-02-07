/* eslint-disable @typescript-eslint/no-unused-vars */
import {type TimelineGroupState, useRobartState} from '@MRAControl/state/useRobartState';

type TimelineGroupProps = {
	group: TimelineGroupState;
};

export const pixelsPerSecond = 25;

export const TimelineGroupBody = ({group}: TimelineGroupProps) => {
	const scale = useRobartState((state) => state.timelineState.scale);
	if (group !== undefined){

	const timelineGroupWidth = (group.duration * pixelsPerSecond) / 1000;
	}
	else{
		const timelineGroupWidth = 0.1;
	}
};
