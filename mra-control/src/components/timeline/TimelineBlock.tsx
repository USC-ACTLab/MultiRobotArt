import { TimelineItem, useRobartState } from '@MRAControl/state/useRobartState';
import { useDrag } from '@use-gesture/react';
import clsx from 'clsx';

import { PIXELS_PER_SECOND, blockOverlaps, convertPixelsToSeconds } from './TimelineGroupBody';

export const TimelineBlock = ({ item, scale }: { item: TimelineItem; scale: number }) => {
  const blocks = useRobartState((state) => state.blocks);
  const selectedBlock = useRobartState((state) => state.editingBlockId);
  const groups = useRobartState((state) => state.timelineState.groups);
  const updateItem = useRobartState((state) => state.updateBlockInTimeline);

  const correspondingBlock = blocks[item.blockId];

  const bind = useDrag(({ delta: [x, _] }) => {
    if (selectedBlock !== undefined) return;

    const seconds_delta = convertPixelsToSeconds(x, scale);
    const newStartTime = item.startTime + seconds_delta;

    if (!blockOverlaps(groups[item.groupId], blocks, newStartTime, correspondingBlock, item.id)) {
      updateItem(item.groupId, item.id, newStartTime);
    }
  });
  return (
    <div
      className={clsx(
        'absolute top-1/2 flex h-5/6 -translate-y-1/2 items-center justify-center rounded-xl bg-ye touch-none select-none',
        selectedBlock === undefined ? 'cursor-move' : '',
      )}
      style={{
        width: PIXELS_PER_SECOND * scale * correspondingBlock.duration,
        left: PIXELS_PER_SECOND * scale * item.startTime,
      }}
      {...bind()}
    >
      <span className="block overflow-hidden text-ellipsis whitespace-nowrap">{correspondingBlock.name}</span>
    </div>
  );
};
