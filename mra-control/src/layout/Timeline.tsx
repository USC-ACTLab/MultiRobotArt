import { TimelineMarker } from '@MRAControl/components/timeline/TimelineMarker';
import { useSimulator } from '@MRAControl/state/useSimulator';
import { faArrowsLeftRight, faEraser, faPause, faPlay, faPlusCircle, faRobot, faSquare } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { Button } from 'flowbite-react';
import { IconButton } from '../components/buttons/IconButton';
import { TimelineGroupBody } from '../components/timeline/TimelineGroupBody';
import { TimelineGroupLabel } from '../components/timeline/TimelineGroupLabel';
import { useRobartState } from '../state/useRobartState';
import { useUIState } from '../state/useUIState';
import { RobotManagerModal } from './robotManager/RobotManagerModal';

import Ruler from "@scena/react-ruler";

export const Timeline = () => {
  const timelineState = useRobartState((state) => state.timelineState);
  const groups = Object.values(timelineState.groups);
  const timelineMode = useRobartState((state) => state.timelineState.mode);
  const setTimelineMode = useRobartState((state) => state.setTimelineMode);
  const toggleRobotManagerModal = useUIState((state) => state.toggleRobotManager);
  const simulationStatus = useSimulator((state) => state.status);
  const play = useSimulator((state) => state.play);
  const halt = useSimulator((state) => state.halt);
  const pause = useSimulator((state) => state.pause);
  const resume = useSimulator((state) => state.resume);

  return (
    <div className="flex h-full w-full flex-col gap-2 rounded bg-blue-100">
      <div className="flex">
        <div className="flex gap-3 pt-2 pl-3">
          <Button.Group>
            <Button color={timelineMode === 'ADD' ? 'info' : 'gray'} onClick={() => setTimelineMode('ADD')}>
              <FontAwesomeIcon icon={faPlusCircle} />
            </Button>
            <Button color={timelineMode === 'MOVE' ? 'info' : 'gray'} onClick={() => setTimelineMode('MOVE')}>
              <FontAwesomeIcon icon={faArrowsLeftRight} />
            </Button>
            <Button color={timelineMode === 'ERASE' ? 'info' : 'gray'} onClick={() => setTimelineMode('ERASE')}>
              <FontAwesomeIcon icon={faEraser} />
            </Button>
          </Button.Group>
        </div>
        
        <div className="flex flex-grow" />
        <div className="flex gap-3 pt-2 pr-3">
          <IconButton icon={faRobot} onClick={toggleRobotManagerModal} text="Manage Robots" />
          <RobotManagerModal />

          {simulationStatus === 'RUNNING' && <IconButton icon={faPause} onClick={pause} text="Pause Sim" color="gray" />}
          {simulationStatus === 'PAUSED' && <IconButton icon={faPlay} onClick={resume} text="Resume Sim" color="success" />}
          {simulationStatus === 'STOPPED' && <IconButton icon={faPlay} onClick={play} text="Run Sim" color="success" />}
          {simulationStatus !== 'STOPPED' && <IconButton icon={faSquare} onClick={halt} text="Stop Sim" color="failure" />}
        </div>
      </div>
      
      <div className="overflow-y-auto">
      {/* <div style={{ padding: 10 }}> */}
          {/* <Ruler textAlign={"center"}
                segment={5}
                mainLineSize={12}
                shortLineSize={5}
                longLineSize={4}
                type="horizontal"
                useResizeObserver={true}
                zoom={25}
                style={{ display: "flex", width: "100%", height: "10%", backgroundColor: "transparent" }}
              /> */}
          {/* </div> */}
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
