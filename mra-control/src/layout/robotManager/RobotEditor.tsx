import { faTrash } from "@fortawesome/free-solid-svg-icons";
import { useEffect, useState } from "react";
import { IconButton } from "@MRAControl/components/buttons/IconButton";
import { useRobartState } from "@MRAControl/state/useRobartState";
import { useRobotManager } from "@MRAControl/state/useRobotManager";
import { StartingPositionEditor } from "./StartingPositionEditor";
import { RobotGroupEditor } from "./RobotGroupEditor";
import { RenamableText } from "@MRAControl/components/utils/RenamableText";

export const RobotEditor = () => {
    const robots = useRobartState((state) => state.robots);
    const updateRobot = useRobartState((state) => state.saveRobot);
    const deleteRobot = useRobartState((state) => state.deleteRobot);

    const selectedRobotId = useRobotManager((state) => state.selectedRobotId);
    const setSelectedRobotId = useRobotManager(
        (state) => state.setSelectedRobotId
    );

    useEffect(() => {
        if (
            selectedRobotId === undefined &&
            Object.values(robots).length !== 0
        ) {
            setSelectedRobotId(Object.keys(robots)[0]);
        }
    }, [selectedRobotId, robots]);

    if (selectedRobotId === undefined) return <></>;

    const selectedRobot = robots[selectedRobotId];

    return (
        <div className="relative flex basis-4/5 flex-col p-4">
            <RenamableText
                text={selectedRobot.name}
                className="h-min w-fit min-w-[10px] gap-4 rounded-md border-2 border-white p-2 text-4xl font-extrabold transition-[width] duration-150 hover:border-black"
                updateText={(newText) =>
                    newText !== ""
                        ? updateRobot(selectedRobotId, { name: newText })
                        : updateRobot(selectedRobotId, {
                              name: "Unnamed Robot",
                          })
                }
            />

            <div className="flex flex-col gap-10">
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
