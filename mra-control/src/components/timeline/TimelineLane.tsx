import React, { useRef } from "react";
import { TimelineLaneState, useRobartState } from "../../state/useRobartState";
import { TimelineBlock } from "./TimelineBlock";

interface TimelineLaneProps {
  lane: TimelineLaneState;
}

// 1 distance unit = 100 pixels = 1 second * scale  = scale 1
export const PIXELS_PER_SECOND = 10;

export const TimelineLane = ({ lane }: TimelineLaneProps) => {
  // Assume the outer is in a vertical flex-col
  const r = useRef<HTMLDivElement>(null);
  const scale = useRobartState((state) => state.timelineState.scale);
  const addBlockToTimeline = useRobartState(
    (state) => state.addBlockToTimeline
  );
  const selectedBlockId = useRobartState((state) => state.editingBlockId);
  const blocks = useRobartState((state) => state.blocks);

  return (
    <div className="flex h-24 flex-row gap-1 rounded">
      <div className="bg-red flex w-16 justify-center rounded bg-green-400">
        <div className="flex flex-col justify-center font-bold">
          {lane.name}
        </div>
      </div>
      <div
        className="relative rounded bg-blue-300"
        style={{ width: 2000 }}
        ref={r}
        onClick={(e) => {
          console.log(e);
          const { clientX, clientY } = e;
          if (r.current) {
            const parentOffsetX = r.current.offsetLeft;
            const parentOffsetY = r.current.offsetTop;
            const offsetX = clientX - parentOffsetX;
            const offsetY = clientY - parentOffsetY;

            const startTime = offsetX / (PIXELS_PER_SECOND * scale);

            // Add the block to the timeline
            if (!selectedBlockId) return;

            const selectedBlock = blocks[selectedBlockId];

            const noOverlap = lane.items.every((items) => {
              const itemStart = items.startTime;
              const itemEnd = items.startTime + blocks[items.blockId].duration;
              const newBlockStart = startTime;
              const newBlockEnd = startTime + selectedBlock.duration;

              return itemEnd < newBlockStart || newBlockEnd < itemStart;
            });

            if (!noOverlap) return;

            addBlockToTimeline(lane.id, selectedBlockId, startTime);
          }
        }}
      >
        {lane.items.map((item, idx) => (
          <TimelineBlock key={idx} scale={scale} item={item} />
        ))}
      </div>
    </div>
  );
};
