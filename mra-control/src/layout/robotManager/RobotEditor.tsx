import { IconButton } from '@MRAControl/components/buttons/IconButton';
import { RenamableText } from '@MRAControl/components/utils/RenamableText';
import { useRobartState } from '@MRAControl/state/useRobartState';
import { useRobotManager } from '@MRAControl/state/useRobotManager';
import { faTrash } from '@fortawesome/free-solid-svg-icons';
import { useEffect } from 'react';

import { RobotGroupEditor } from './RobotGroupEditor';
import { StartingPositionEditor } from './StartingPositionEditor';

export const RobotEditor = () => {
  const robots = useRobartState((state) => state.robots);
  const updateRobot = useRobartState((state) => state.saveRobot);
  const deleteRobot = useRobartState((state) => state.deleteRobot);

  const selectedRobotId = useRobotManager((state) => state.selectedRobotId);
  const setSelectedRobotId = useRobotManager((state) => state.setSelectedRobotId);

  useEffect(() => {
    if (selectedRobotId === undefined && Object.values(robots).length !== 0) {
      setSelectedRobotId(Object.keys(robots)[0]);
    }
  }, [selectedRobotId, robots]);

  if (selectedRobotId === undefined) return <></>;

  const selectedRobot = robots[selectedRobotId];

  return (
    <div className="relative flex basis-4/5 flex-col p-4">
      <RenamableText
        text={selectedRobot.name}
        className={'text-4xl font-extrabold'}
        updateText={(newText) =>
          newText !== ''
            ? updateRobot(selectedRobotId, { name: newText })
            : updateRobot(selectedRobotId, {
                name: 'Unnamed Robot',
              })
        }
      />

      <div className="flex flex-col gap-10" key={selectedRobotId}>
        <StartingPositionEditor robotId={selectedRobotId} />
        <RobotGroupEditor />
      </div>

      <IconButton
        className="absolute bottom-0"
        onClick={() => {
          const currSelectedRobotId = selectedRobotId;
          setSelectedRobotId(undefined);
          deleteRobot(currSelectedRobotId);
        }}
        text="Delete Robot"
        icon={faTrash}
        color="failure"
      />
    </div>
  );
};
