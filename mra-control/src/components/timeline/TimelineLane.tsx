import React from "react";
import { TimelineLaneState, useRobartState } from "../../state/useRobartState";

interface TimelineLaneProps {
  lane: TimelineLaneState;
}

export const TimelineLane = ({ lane }: TimelineLaneProps) => {
  // Assume the outer is in a vertical flex-col
  return (
    <div className="flex h-24 flex-row gap-1 rounded">
      <div className="bg-red flex w-16 justify-center rounded bg-green-400">
        <div className="flex flex-col justify-center font-bold">
          {lane.name}
        </div>
      </div>
      <div className="flex rounded bg-blue-300" style={{ width: 2000 }}>
        Some blocks
      </div>
    </div>
  );
};
