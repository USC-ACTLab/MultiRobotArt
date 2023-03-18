import {
    faArrowsRotate,
    faCheck,
    faPlusCircle,
    faTrash,
} from "@fortawesome/free-solid-svg-icons";
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import { Badge, Dropdown, Label, TextInput } from "flowbite-react";
import { useState } from "react";
import { IconButton } from "../../components/buttons/IconButton";
import { useRobartState } from "../../state/useRobartState";
import { useRobotManager } from "../../state/useRobotManager";

export const RobotEditor = () => {
    const robots = useRobartState((state) => state.robots);
    const updateRobot = useRobartState((state) => state.saveRobot);
    const deleteRobot = useRobartState((state) => state.deleteRobot);
    const addRobotToGroup = useRobartState((state) => state.addRobotToGroup);
    const groups = useRobartState((state) => state.timelineState.groups);
    const selectedRobotId = useRobotManager((state) => state.selectedRobotId);
    const setSelectedRobotId = useRobotManager(
        (state) => state.setSelectedRobotId
    );
    const showRenameInput = useRobotManager((state) => state.showRenameInput);
    const openRenameInput = useRobotManager((state) => state.openRenameInput);
    const closeRenameInput = useRobotManager((state) => state.closeRenameInput);
    const [inputValue, setInputValue] = useState("");

    if (selectedRobotId === undefined) return <></>;

    const selectedRobot = robots[selectedRobotId];

    return (
        <div className="relative flex basis-4/5 flex-col p-4">
            {showRenameInput ? (
                <div className="flex h-min w-full items-center gap-4 text-4xl font-extrabold">
                    <form
                        onSubmit={(e) => {
                            e.preventDefault();
                            updateRobot(selectedRobotId, {
                                name: inputValue,
                            });
                            closeRenameInput();
                        }}
                    >
                        <TextInput
                            className="w-full"
                            sizing="4xl"
                            value={inputValue}
                            onChange={(e) => setInputValue(e.target.value)}
                            autoFocus
                        />
                        <input type="submit" className="hidden" />
                    </form>
                    <FontAwesomeIcon
                        icon={faCheck}
                        className="cursor-pointer text-xl text-green-500"
                        onClick={() => {
                            updateRobot(selectedRobotId, {
                                name: inputValue,
                            });
                            closeRenameInput();
                        }}
                    />
                    <FontAwesomeIcon
                        icon={faTrash}
                        className="cursor-pointer text-xl text-red-500"
                        onClick={() => {
                            closeRenameInput();
                        }}
                    />
                </div>
            ) : (
                <h2
                    className="flex h-min w-fit items-center gap-4 rounded-md border-2 border-white p-2  text-4xl font-extrabold hover:border-black"
                    onClick={() => {
                        setInputValue(selectedRobot.name);
                        openRenameInput();
                    }}
                >
                    {selectedRobot.name}
                </h2>
            )}

            <div className="flex flex-col gap-10">
                <div>
                    <h3 className="ml-3 mt-3 text-lg font-extrabold">
                        Starting Position:
                    </h3>
                    <div className="flex gap-2">
                        <form>
                            <div className="flex flex-col items-center">
                                <TextInput id="x-coordinate" />
                                <Label htmlFor="x-coordinate">x</Label>
                            </div>

                            <div className="flex flex-col items-center">
                                <TextInput id="y-coordinate" />
                                <Label htmlFor="y-coordinate">y</Label>
                            </div>

                            <input className="hidden" type="submit" />
                        </form>
                        <IconButton
                            icon={faArrowsRotate}
                            text="Update Position"
                        />
                    </div>
                </div>
                <div>
                    <div className="flex items-center gap-2">
                        <h3 className="text-lg font-extrabold">Add Group</h3>
                        <Dropdown
                            label={<FontAwesomeIcon icon={faPlusCircle} />}
                            size="sm"
                            arrowIcon={false}
                        >
                            {Object.values(groups)
                                .filter(
                                    (group) =>
                                        !selectedRobot.groups.has(group.id)
                                )
                                .map((group) => (
                                    <Dropdown.Item
                                        key={group.id}
                                        onClick={() => {
                                            addRobotToGroup(
                                                group.id,
                                                selectedRobotId
                                            );
                                        }}
                                    >
                                        {group.name}
                                    </Dropdown.Item>
                                ))}
                        </Dropdown>
                    </div>
                    <div className="flex items-center">
                        {[...selectedRobot.groups].map((groupId) => {
                            return (
                                <Badge
                                    color="purple"
                                    className="bg-purple-100"
                                    key={groupId}
                                >
                                    {groups[groupId].name}
                                </Badge>
                            );
                        })}
                    </div>
                </div>
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
