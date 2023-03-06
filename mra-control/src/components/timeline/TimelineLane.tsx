import React from "react";
import { TimelineLaneState, useMRAState } from "../../state/useMRAState";

interface TimelineLaneProps {
  lane: TimelineLaneState;
}

export const TimelineLane = ({ lane }: TimelineLaneProps) => {
  // Assume the outer is in a vertical flex-col
  return (
    <div className="flex w-full bg-blue-300 h-16 rounded">{lane.name}</div>
  );
};
