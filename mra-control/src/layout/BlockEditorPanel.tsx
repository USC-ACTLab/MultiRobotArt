import React, { useEffect, useRef, useState } from "react";
import { useBlocklyWorkspace } from "react-blockly";
import { blocklyToolboxConfiguration } from "../config/BlockToolboxConfig";
import { useBlockEditorState } from "../state/useBlockEditorState";
import { pythonGenerator } from "blockly/python";
import { CodeBlock, useRobartState } from "../state/useMRAState";
import Blockly from "blockly";
import "../config/customBlocks";
import { Button } from "flowbite-react";
import { RenameBlockModal } from "../components/blocks/RenameBlockModal";

export const BlockEditorPanel = () => {
  const workspaceRef = useRef<any>(null);
  const currentBlockId = useRobartState((state) => state.editingBlockId);
  const currentBlock: CodeBlock | undefined = useRobartState(
    (state) => state.blocks[currentBlockId ?? ""]
  );
  const setBlocklyPython = useBlockEditorState(
    (state) => state.setBlocklyPython
  );
  const setBlocklyXML = useBlockEditorState((state) => state.setBlocklyXML);
  const saveBlock = useRobartState((state) => state.saveBlock);

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
    <div className="h-full w-full">
      {!currentBlock && <div className="m-2">No Block selected.</div>}
      {currentBlock && (
        <div className="m-2 flex flex-row gap-2">
          <div className="flex text-lg font-bold">
            Editing Block {currentBlock.name}
          </div>
          <Button
            className="flex"
            color="success"
            onClick={() =>
              saveBlock(currentBlock.id, {
                name: currentBlock.name,
                xml: xml ?? "",
              })
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
      <div ref={workspaceRef} className="h-full w-full"></div>
    </div>
  );
};
