import { faPlusCircle } from '@fortawesome/free-solid-svg-icons';

import { IconButton } from '../../components/buttons/IconButton';
import { useRobartState } from '../../state/useRobartState';
import { useRobotManager } from '../../state/useRobotManager';

export const RobotSidebar = () => {
  const robots = useRobartState((state) => state.robots);
  const createRobot = useRobartState((state) => state.createRobot);
  const setSelectedRobotId = useRobotManager((state) => state.setSelectedRobotId);
  const closeRenameInput = useRobotManager((state) => state.closeRenameInput);

  return (
    <div className="flex flex-grow-0 basis-[10%] flex-col items-center gap-2 overflow-y-auto p-4">
      <h2 className="text-lg font-extrabold">Robots</h2>
      {Object.values(robots).map((robot) => (
        <button
          key={robot.id}
          className="flex h-8 w-full cursor-pointer items-center justify-center rounded-lg border-2 border-black px-2 text-center"
          onClick={() => {
            closeRenameInput();
            setSelectedRobotId(robot.id);
          }}
        >
          <span className="overflow-hidden overflow-ellipsis whitespace-nowrap">{robot.name}</span>
        </button>
      ))}
      <IconButton icon={faPlusCircle} text="" onClick={createRobot} />
    </div>
  );
};
