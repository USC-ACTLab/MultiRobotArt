import { Button } from "flowbite-react";
import React, { useState } from "react";
import { UploadFileModal } from "../components/modal/UploadModal";
import { useRobartState } from "../state/useMRAState";
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import {
  faDownload,
  faUpload,
  faGear,
} from "@fortawesome/free-solid-svg-icons";
import { IconButton } from "../components/buttons/IconButton";
import { useUIState } from "../state/useUIState";
import { SettingsModal } from "./settings/SettingsModal";

export const NavigationBar = () => {
  const [loadModalOpen, setLoadModalOpen] = useState(false);
  const saveToFile = useRobartState((state) => state.saveToFile);
  const loadFile = useRobartState((state) => state.loadFile);
  const toggleSettingsModal = useUIState((state) => state.toggleSettingsModal);
  return (
    <>
      <div className="flex items-center gap-5 border-b-2 border-black p-4">
        <div className="text-4xl font-extrabold">Robart</div>
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
