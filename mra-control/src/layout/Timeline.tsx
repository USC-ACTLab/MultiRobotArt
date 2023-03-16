import { faPlay, faRobot } from "@fortawesome/free-solid-svg-icons";
import React from "react";
import { IconButton } from "../components/buttons/IconButton";
import { TimelineGroupBody } from "../components/timeline/TimelineGroupBody";
import { TimelineGroupLabel } from "../components/timeline/TimelineGroupLabel";
import { useRobartState } from "../state/useRobartState";

export const Timeline = () => {
    const timelineState = useRobartState((state) => state.timelineState);
    const groups = Object.values(timelineState.groups);

    return (
        <div className="flex h-full w-full flex-col gap-2 rounded bg-blue-100">
            <div className="flex justify-end gap-3 pt-2 pr-3">
                <IconButton icon={faRobot} text="Manage Robots" />
                <IconButton icon={faPlay} text="Run Sim" color="success" />
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
