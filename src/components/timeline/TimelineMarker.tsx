import { useSimulator } from "@MRAControl/state/useSimulator";
import { PIXELS_PER_SECOND } from "./TimelineGroupBody";

export const TimelineMarker = () => {
  const time = useSimulator((state) => state.time);
  return (
    <div
      className="min-w-1 absolute z-10 h-full w-1 bg-black"
      style={{ left: PIXELS_PER_SECOND * time }}
    ></div>
  );
};
