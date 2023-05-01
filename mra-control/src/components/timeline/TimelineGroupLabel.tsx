import { TimelineGroupState } from '../../state/useRobartState';

interface TimelineGroupProps {
  group: TimelineGroupState;
}

export const TimelineGroupLabel = ({ group }: TimelineGroupProps) => {
  return (
    <div className="flex h-12 w-24 items-center justify-center rounded-xl bg-bl">
      <div className="text-center text-white">{group.name}</div>
    </div>
  );
};
