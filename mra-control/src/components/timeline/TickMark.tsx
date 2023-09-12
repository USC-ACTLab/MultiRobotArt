import clsx from 'clsx';

import { convertSecondsToPixels } from './TimelineGroupBody';

interface TickMarkProps {
  tickNumber: number;
  subdivisionsPerSecond: number;
  scale: number;
}

export const TickMark = ({ tickNumber, subdivisionsPerSecond, scale }: TickMarkProps) => {
  return (
    <div className="absolute h-full" style={{ left: `${convertSecondsToPixels(tickNumber / subdivisionsPerSecond, scale)}px` }}>
      <div className={clsx('w-[2px] bg-black self-start', tickNumber % subdivisionsPerSecond === 0 ? 'h-1/4' : 'h-1/6')} />
      {tickNumber % subdivisionsPerSecond === 0 ? <span>{tickNumber / subdivisionsPerSecond}</span> : null}
    </div>
  );
};
