import { faPlay, faRobot, faSquare, faPause } from '@fortawesome/free-solid-svg-icons';
import { IconButton } from '../components/buttons/IconButton';
import { TimelineGroupBody } from '../components/timeline/TimelineGroupBody';
import { TimelineGroupLabel } from '../components/timeline/TimelineGroupLabel';
import { useRobartState } from '../state/useRobartState';
import { useUIState } from '../state/useUIState';
import { RobotManagerModal } from './robotManager/RobotManagerModal';
import { SimulationPanel } from './simulation/SimulationPanel';
import { TimelineMarker } from '@MRAControl/components/timeline/TimelineMarker';
import { useSimulator } from '@MRAControl/state/useSimulator';

export const Timeline = () => {
  const timelineState = useRobartState((state) => state.timelineState);
  const groups = Object.values(timelineState.groups);
  const toggleRobotManagerModal = useUIState((state) => state.toggleRobotManager);
  const simulationStatus = useSimulator((state) => state.status);
  const play = useSimulator(state => state.play);
  const halt = useSimulator(state => state.halt);
  const pause = useSimulator(state => state.pause);
  const resume = useSimulator(state => state.resume);

  return (
    <div className="flex h-full w-full flex-col gap-2 rounded bg-blue-100">
      <div className="flex justify-end gap-3 pt-2 pr-3">
        <IconButton icon={faRobot} onClick={toggleRobotManagerModal} text="Manage Robots" />
        <RobotManagerModal />

        {simulationStatus === 'RUNNING' && (
          <IconButton icon={faPause} onClick={pause} text="Pause Sim" color="gray" />
        )}
        {simulationStatus === 'PAUSED' && (
          <IconButton icon={faPlay} onClick={resume} text="Resume Sim" color="success" />
        )}
        {simulationStatus === 'STOPPED' && (
          <IconButton icon={faPlay} onClick={play} text="Run Sim" color="success" />
        )}
        {simulationStatus !== 'STOPPED' && (
          <IconButton icon={faSquare} onClick={halt} text="Stop Sim" color="failure" />
        )}
      </div>
      <div className="overflow-y-auto">
        <div className="flex flex-shrink-0 gap-2">
          <div className="ml-2 flex h-full flex-col gap-2">
            {groups.map((group) => (
              <TimelineGroupLabel group={group} key={group.id} />
            ))}
          </div>
          <div className="relative flex h-full flex-col gap-2 overflow-x-auto">
            {groups.map((group) => (
              <TimelineGroupBody group={group} key={group.id} />
            ))}
            <TimelineMarker />
          </div>
        </div>
      </div>
    </div>
  );
};
