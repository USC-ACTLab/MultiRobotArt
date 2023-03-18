import { useEffect } from "react";
import { useRobartState } from "../../state/useRobartState";
import { useRobotManager } from "../../state/useRobotManager";
import { RobotEditor } from "./RobotEditor";
import { RobotSidebar } from "./RobotSidebar";

export const RobotManager = () => {
    const robots = useRobartState((state) => state.robots);
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

    return (
        <div className="flex h-full">
            <RobotSidebar />
            <RobotEditor />
        </div>
    );
};
