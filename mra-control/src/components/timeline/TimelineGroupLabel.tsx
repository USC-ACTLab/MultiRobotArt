import { TimelineGroupState } from '../../state/useRobartState';

interface TimelineGroupProps {
  group: TimelineGroupState;
}

export const TimelineGroupLabel = ({ group }: TimelineGroupProps) => {
  return (
    <div className="flex h-16 w-16 items-center justify-center rounded bg-green-400">
      <div className="text-center font-bold">{group.name}</div>
    </div>
  );
};
