import React from "react";
import { TimelineGroup } from "../components/timeline/TimelineGroup";
import { useRobartState } from "../state/useRobartState";

export const Timeline = () => {
  const timelineState = useRobartState((state) => state.timelineState);
  const groups = Object.values(timelineState.groups);

  return (
    <div className="flex h-full w-full flex-col gap-2 overflow-x-auto overflow-y-auto rounded bg-blue-100">
      <div className="flex w-full flex-wrap gap-1 p-4">
        {groups.map((group, i) => (
          <TimelineGroup group={group} key={i} />
        ))}
      </div>
    </div>
  );
};
