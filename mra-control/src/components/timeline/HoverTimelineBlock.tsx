import { faPlusCircle, faXmarkCircle } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import clsx from 'clsx';

import { TimelineItem, useRobartState } from '../../state/useRobartState';
import { PIXELS_PER_SECOND } from './TimelineGroupBody';

export const HoverTimelineBlock = ({ scale, startTime, isOverlapping }: { scale: number; startTime: number | undefined; isOverlapping: boolean }) => {
  const selectedBlockId = useRobartState((state) => state.editingBlockId);
  const selectedBlock = useRobartState((state) => state.blocks[selectedBlockId ?? '']);

  if (startTime === undefined || !selectedBlockId) return <></>;

  return (
    <div
      className={clsx(
        'absolute top-1/2 flex h-5/6 -translate-y-1/2 items-center justify-center rounded-xl border border-dashed',
        isOverlapping ? 'border-red-500/50 bg-red-300/50 text-red-500/50' : 'border-green-500/50 bg-green-300/50 text-green-500/50',
      )}
      style={{
        width: PIXELS_PER_SECOND * scale * selectedBlock.duration,
        left: PIXELS_PER_SECOND * scale * startTime!,
      }}
    >
      {isOverlapping ? <FontAwesomeIcon icon={faXmarkCircle} /> : <FontAwesomeIcon icon={faPlusCircle} />}
    </div>
  );
};
