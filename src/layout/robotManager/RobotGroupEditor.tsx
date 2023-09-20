import { useRobartState } from '@MRAControl/state/useRobartState';
import { useRobotManager } from '@MRAControl/state/useRobotManager';
import { faPlusCircle } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { Badge, Dropdown } from 'flowbite-react';

export const RobotGroupEditor = () => {
  const groups = useRobartState((state) => state.timelineState.groups);
  const selectedRobotId = useRobotManager((state) => state.selectedRobotId);
  const addRobotToGroup = useRobartState((state) => state.addRobotToGroup);
  const removeRobotFromGroup = useRobartState((state) => state.removeRobotFromGroup);

  if (selectedRobotId === undefined) return <></>;

  return (
    <div className="flex flex-col gap-2">
      <div className="flex items-center gap-2">
        <h3 className="text-lg font-extrabold">Add Group</h3>
        <Dropdown label={<FontAwesomeIcon icon={faPlusCircle} />} size="sm" arrowIcon={false}>
          {Object.values(groups)
            .filter((group) => !(selectedRobotId in group.robots))
            .map((group) => (
              <Dropdown.Item
                key={group.id}
                onClick={() => {
                  addRobotToGroup(group.id, selectedRobotId);
                }}
              >
                {group.name}
              </Dropdown.Item>
            ))}
        </Dropdown>
      </div>
      <div className="flex items-center gap-2">
        {Object.values(groups)
          .filter((group) => selectedRobotId in group.robots)
          .map((group) => {
            return (
              <Badge color="purple" className="bg-purple-100" onClick={() => removeRobotFromGroup(group.id, selectedRobotId)} key={group.id}>
                {groups[group.id].name}
              </Badge>
            );
          })}
      </div>
    </div>
  );
};
