import React, { useRef } from "react";
import {
    CodeBlock,
    TimelineGroupState,
    useRobartState,
} from "../../state/useRobartState";
import { TimelineBlock } from "./TimelineBlock";

interface TimelineGroupProps {
    group: TimelineGroupState;
}

// 1 distance unit = 100 pixels = 1 second * scale  = scale 1
export const PIXELS_PER_SECOND = 10;

export const TimelineGroupBody = ({ group }: TimelineGroupProps) => {
    // Assume the outer is in a vertical flex-col
    const laneBodyRef = useRef<HTMLDivElement>(null);
    const scale = useRobartState((state) => state.timelineState.scale);
    const addBlockToTimeline = useRobartState(
        (state) => state.addBlockToTimeline
    );
    const selectedBlockId = useRobartState((state) => state.editingBlockId);
    const blocks = useRobartState((state) => state.blocks);

    const blockOverlaps = (startTime: number, selectedBlock: CodeBlock) =>
        group.items.some((items) => {
            const currItemStart = items.startTime;
            const currItemEnd =
                items.startTime + blocks[items.blockId].duration;
            const newBlockStart = startTime;
            const newBlockEnd = startTime + selectedBlock.duration;

            return (
                newBlockStart < 0 ||
                !(currItemEnd < newBlockStart || newBlockEnd < currItemStart)
            );
        });

    const computeTimelineBlockOffset = (
        e: React.MouseEvent<HTMLDivElement>
    ) => {
        const { clientX } = e;
        if (laneBodyRef.current) {
            const parentOffsetX = laneBodyRef.current.offsetLeft;
            const parentScrollOffsetX =
                laneBodyRef.current.parentElement?.scrollLeft;
            const offsetX = clientX - parentOffsetX;

            // Add the block to the timeline
            if (
                selectedBlockId === undefined ||
                parentScrollOffsetX === undefined
            )
                return;

            const startTime =
                (offsetX + parentScrollOffsetX) / (PIXELS_PER_SECOND * scale) -
                blocks[selectedBlockId].duration / 2;

            return startTime;
        }
    };

    return (
        <div
            className="relative h-16 rounded bg-blue-300"
            style={{ width: 2000 }}
            ref={laneBodyRef}
            onClick={(e) => {
                if (selectedBlockId === undefined) return;

                console.log("ehre?");

                const startTime = computeTimelineBlockOffset(e);

                if (
                    startTime &&
                    !blockOverlaps(startTime, blocks[selectedBlockId])
                )
                    addBlockToTimeline(group.id, selectedBlockId, startTime);
            }}
        >
            {group.items.map((item, idx) => (
                <TimelineBlock key={idx} scale={scale} item={item} />
            ))}
        </div>
    );
};
