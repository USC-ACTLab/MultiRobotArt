import { Button } from "flowbite-react";
import React, { useState } from "react";
import { UploadFileModal } from "@MRAControl/components/modal/UploadModal";
import { useRobartState } from "@MRAControl/state/useRobartState";
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import {
    faDownload,
    faUpload,
    faGear,
} from "@fortawesome/free-solid-svg-icons";
import { IconButton } from "@MRAControl/components/buttons/IconButton";
import { useUIState } from "@MRAControl/state/useUIState";
import { RenamableText } from "@MRAControl/components/utils/RenamableText";
import { SettingsModal } from "./settings/SettingsModal";

export const NavigationBar = () => {
    const [loadModalOpen, setLoadModalOpen] = useState(false);
    const saveToFile = useRobartState((state) => state.saveProject);
    const loadFile = useRobartState((state) => state.loadProject);
    const toggleSettingsModal = useUIState(
        (state) => state.toggleSettingsModal
    );
    const projectName = useRobartState((state) => state.projectName);
    const setProjectName = useRobartState((state) => state.setProjectName);

    return (
        <>
            <div className="flex items-center gap-5 border-b-2 border-black p-4">
                <RenamableText
                    className="text-4xl font-extrabold"
                    text={projectName}
                    updateText={(newText) => {
                        newText !== ""
                            ? setProjectName(newText)
                            : setProjectName("New Robart Project");
                    }}
                />
                <div className="flex flex-grow justify-end gap-3">
                    <IconButton
                        icon={faDownload}
                        text="Save Project"
                        onClick={() => saveToFile()}
                    />
                    <IconButton
                        icon={faUpload}
                        text="Load Project"
                        onClick={() => setLoadModalOpen(true)}
                    />
                    <IconButton
                        icon={faGear}
                        text="Settings"
                        onClick={() => toggleSettingsModal()}
                    />
                </div>
            </div>
            <UploadFileModal
                header="Load Project"
                open={loadModalOpen}
                onClose={() => setLoadModalOpen(false)}
                onFileUpload={loadFile}
            />
            <SettingsModal />
        </>
    );
};
