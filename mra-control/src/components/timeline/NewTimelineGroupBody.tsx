import { TimelineGroupState, useRobartState } from '@MRAControl/state/useRobartState';

interface TimelineGroupProps {
  group: TimelineGroupState;
}

export const PIXELS_PER_SECOND = 25;

export const TimelineGroupBody = ({ group }: TimelineGroupProps) => {
  const scale = useRobartState((state) => state.timelineState.scale);

  const timelineGroupWidth = (group.duration * PIXELS_PER_SECOND) / 1000;
};
