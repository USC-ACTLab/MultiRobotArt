import React, { useEffect, useRef, useState } from "react";
import { useBlocklyWorkspace } from "react-blockly";
import { blocklyToolboxConfiguration } from "../config/BlockToolboxConfig";
import { useBlockEditorState } from "../state/useBlockEditorState";
import { pythonGenerator } from "blockly/python";
import { useMRAState } from "../state/useMRAState";
import Blockly from "blockly";
import "../config/customBlocks";
import { Button } from "flowbite-react";
import { RenameBlockModal } from "../components/blocks/RenameBlockModal";

export const BlockEditorPanel = () => {
  const workspaceRef = useRef<any>(null);
  const currentBlock = useMRAState((state) => state.editingBlock);
  const setBlocklyPython = useBlockEditorState(
    (state) => state.setBlocklyPython
  );
  const setBlocklyXML = useBlockEditorState((state) => state.setBlocklyXML);
  const saveBlock = useMRAState((state) => state.saveBlock);

  const [renameModalOpen, setRenameModalOpen] = useState<boolean>(false);

  const { workspace, xml } = useBlocklyWorkspace({
    toolboxConfiguration: blocklyToolboxConfiguration,
    initialXml: currentBlock?.xml ?? "",
    workspaceConfiguration: {
      grid: {
        spacing: 20,
        length: 3,
        colour: "#ccc",
        snap: true,
      },
    },
    onWorkspaceChange: (workspace) => {
      const code = pythonGenerator.workspaceToCode(workspace);
      setBlocklyPython(code);
      if (xml) setBlocklyXML(xml);
    },
    ref: workspaceRef,
  });

  useEffect(() => {
    if (currentBlock && currentBlock.xml && workspace) {
      var xmlDom = Blockly.Xml.textToDom(currentBlock.xml);
      Blockly.Xml.clearWorkspaceAndLoadFromXml(xmlDom, workspace);
    } else if (currentBlock && !currentBlock.xml && workspace) {
      workspace.clear();
    }
  }, [currentBlock]);

  return (
    <div className="w-full h-full">
      {!currentBlock && <div className="m-2">No Block selected.</div>}
      {currentBlock && (
        <div className="flex flex-row gap-2 m-2">
          <div className="flex font-bold text-lg">
            Editing Block {currentBlock.name} ({currentBlock.id})
          </div>
          <Button
            className="flex"
            color="success"
            onClick={() =>
              saveBlock(currentBlock.id, currentBlock.name, xml ?? "")
            }
          >
            Save
          </Button>
          <Button
            className="flex"
            color="danger"
            onClick={() => setRenameModalOpen(true)}
          >
            Rename
          </Button>
          <RenameBlockModal
            open={renameModalOpen}
            block={currentBlock}
            onClose={() => setRenameModalOpen(false)}
          />
        </div>
      )}
      <div ref={workspaceRef} className="w-full h-full"></div>
    </div>
  );
};
