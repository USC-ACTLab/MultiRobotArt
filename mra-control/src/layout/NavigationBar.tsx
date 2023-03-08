import React, { useState } from "react";
import { UploadFileModal } from "../components/modal/UploadModal";
import { useRobartState } from "../state/useMRAState";

export const NavigationBar = () => {
  const [loadModalOpen, setLoadModalOpen] = useState(false);
  const saveToFile = useRobartState((state) => state.saveToFile);
  const loadFile = useRobartState((state) => state.loadFile);
  return (
    <>
      <div className="flex h-8 flex-row gap-2 border-b-2 border-black">
        <div className="flex font-extrabold">Robart</div>
        <div className="flex font-bold" onClick={() => saveToFile(undefined)}>
          Save
        </div>
        <div className="flex font-bold" onClick={(e) => setLoadModalOpen(true)}>
          Load
        </div>
      </div>
      <UploadFileModal
        header="Load Project"
        open={loadModalOpen}
        onClose={() => setLoadModalOpen(false)}
        onFileUpload={loadFile}
      />
    </>
  );
};
