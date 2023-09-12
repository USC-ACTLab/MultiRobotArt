import { IconButton } from '@MRAControl/components/buttons/IconButton';
import { useSimulator } from '@MRAControl/state/useSimulator';
import { faPause, faPlay, faSquare } from '@fortawesome/free-solid-svg-icons';

export const TimelineSimulationButtons = () => {
  const simulationStatus = useSimulator((state) => state.status);
  const play = useSimulator((state) => state.play);
  const halt = useSimulator((state) => state.halt);
  const pause = useSimulator((state) => state.pause);
  const resume = useSimulator((state) => state.resume);

  return (
    <>
      {simulationStatus === 'RUNNING' && <IconButton icon={faPause} onClick={pause} text="Pause Sim" color="gray" />}
      {simulationStatus === 'PAUSED' && <IconButton icon={faPlay} onClick={resume} text="Resume Sim" color="success" />}
      {simulationStatus === 'STOPPED' && <IconButton icon={faPlay} onClick={play} text="Run Sim" color="success" />}
      {simulationStatus !== 'STOPPED' && <IconButton icon={faSquare} onClick={halt} text="Stop Sim" color="failure" />}
    </>
  );
};
