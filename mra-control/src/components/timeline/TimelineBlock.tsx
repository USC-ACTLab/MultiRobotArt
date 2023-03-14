import { TimelineItem, useRobartState } from "../../state/useRobartState";

import { PIXELS_PER_SECOND } from "./TimelineGroup";

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
      className="absolute top-1/2 h-5/6 -translate-y-1/2 rounded-xl bg-purple-400 text-center"
      style={{
        width: PIXELS_PER_SECOND * scale * block.duration,
        left: PIXELS_PER_SECOND * scale * item.startTime,
      }}
    >
      {block.name}
    </div>
  );
};
