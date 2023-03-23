import { faPlay, faRobot, faSquare } from '@fortawesome/free-solid-svg-icons';

import { IconButton } from '../components/buttons/IconButton';
import { TimelineGroupBody } from '../components/timeline/TimelineGroupBody';
import { TimelineGroupLabel } from '../components/timeline/TimelineGroupLabel';
import { useRobartState } from '../state/useRobartState';
import { useUIState } from '../state/useUIState';
import { RobotManagerModal } from './robotManager/RobotManagerModal';

export const Timeline = () => {
  const timelineState = useRobartState((state) => state.timelineState);
  const groups = Object.values(timelineState.groups);
  const toggleRobotManagerModal = useUIState((state) => state.toggleRobotManager);
  const openSimulation = useUIState((state) => state.openSimulation);
  const toggleSimulation = useUIState((state) => state.toggleSimulation);

  return (
    <div className="flex h-full w-full flex-col gap-2 rounded bg-blue-100">
      <div className="flex justify-end gap-3 pt-2 pr-3">
        <IconButton icon={faRobot} onClick={toggleRobotManagerModal} text="Manage Robots" />
        <RobotManagerModal />

        {openSimulation ? (
          <IconButton icon={faSquare} onClick={toggleSimulation} text="Stop Sim" color="failure" />
        ) : (
          <IconButton icon={faPlay} onClick={toggleSimulation} text="Run Sim" color="success" />
        )}
      </div>
      <div className="overflow-y-auto">
        <div className="flex flex-shrink-0 gap-2">
          <div className="ml-2 flex h-full flex-col gap-2">
            {groups.map((group) => (
              <TimelineGroupLabel group={group} key={group.id} />
            ))}
          </div>
          <div className="flex h-full flex-col gap-2 overflow-x-auto">
            {groups.map((group) => (
              <TimelineGroupBody group={group} key={group.id} />
            ))}
          </div>
        </div>
      </div>
    </div>
  );
};
