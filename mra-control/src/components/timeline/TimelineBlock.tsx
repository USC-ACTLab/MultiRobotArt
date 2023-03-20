import { TimelineItem, useRobartState } from "../../state/useRobartState";

import { PIXELS_PER_SECOND } from "./TimelineGroupBody";

export const TimelineBlock = ({
    item,
    scale,
}: {
    item: TimelineItem;
    scale: number;
}) => {
    const block = useRobartState((state) => state.blocks[item.blockId]);
    return (
        <div
            className="absolute top-1/2 flex h-5/6 -translate-y-1/2 items-center justify-center rounded-xl bg-purple-400"
            style={{
                width: PIXELS_PER_SECOND * scale * block.duration,
                left: PIXELS_PER_SECOND * scale * item.startTime,
            }}
        >
            <span className="block overflow-hidden text-ellipsis whitespace-nowrap">
                {block.name}
            </span>
        </div>
    );
};
