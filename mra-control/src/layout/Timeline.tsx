import React from "react";
import { TimelineLane } from "../components/timeline/TimelineLane";
import { useRobartState } from "../state/useRobartState";

export const Timeline = () => {
  const timelineState = useRobartState((state) => state.timelineState);
  const lanes = Object.values(timelineState.lanes);

  return (
    <div className="flex h-full w-full flex-col gap-2 overflow-x-auto overflow-y-auto rounded bg-blue-100">
      <div className="flex w-full flex-wrap gap-1 p-4">
        {lanes.map((lane, i) => (
          <TimelineLane lane={lane} key={i} />
        ))}
      </div>
    </div>
  );
};
