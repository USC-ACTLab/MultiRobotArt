import { TimelineItem, useRobartState } from '@MRAControl/state/useRobartState';
import { useDrag } from '@use-gesture/react';
import clsx from 'clsx';

import { PIXELS_PER_SECOND, blockOverlaps, convertPixelsToSeconds } from './TimelineGroupBody';

export const TimelineBlock = ({ item, scale }: { item: TimelineItem; scale: number }) => {
  const blocks = useRobartState((state) => state.blocks);
  const removeItem = useRobartState((state) => state.removeTimelineItem);
  const groups = useRobartState((state) => state.timelineState.groups);
  const updateItem = useRobartState((state) => state.updateBlockInTimeline);
  const timelineMode = useRobartState((state) => state.timelineState.mode);

  const correspondingBlock = blocks[item.blockId];

  const bind = useDrag(({ delta: [x, _] }) => {
    if (timelineMode !== 'MOVE') return;

    const seconds_delta = convertPixelsToSeconds(x, scale);
    const newStartTime = item.startTime + seconds_delta;

    if (!blockOverlaps(groups[item.groupId], blocks, newStartTime, correspondingBlock, )) {
      updateItem(item.groupId, item.id, newStartTime);
    }
  });
  const duration = item.duration;
  return (
    <div
      className={clsx(
        'absolute top-1/2 flex h-5/6 -translate-y-1/2 items-center justify-center rounded-xl bg-purple-400 touch-none select-none',
        timelineMode === 'MOVE' ? 'cursor-move' : '',
        timelineMode === 'ERASE' ? 'hover:bg-red-400' : '',
      )}
      style={{
        width: PIXELS_PER_SECOND * scale * duration,
        left: PIXELS_PER_SECOND * scale * item.startTime,
      }}
      onClick={() => timelineMode === 'ERASE' && removeItem(item.groupId, item.id)}
      {...bind()}
    >
      <span className="block overflow-hidden text-ellipsis whitespace-nowrap">{correspondingBlock.name}</span>
    </div>
  );
};
