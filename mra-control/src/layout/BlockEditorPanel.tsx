import React, { useEffect, useRef } from "react";
import { useBlocklyWorkspace } from "react-blockly";
import { blocklyToolboxConfiguration } from "../config/BlockToolboxConfig";
import { useBlockEditorState } from "../state/useBlockEditorState";
import { pythonGenerator } from "blockly/python";
import { useMRAState } from "../state/useMRAState";
import Blockly from "blockly";
import "../config/customBlocks";

export const BlockEditorPanel = () => {
  const workspaceRef = useRef<any>(null);
  const currentBlock = useMRAState((state) => state.editingBlock);
  const setBlocklyPython = useBlockEditorState(
    (state) => state.setBlocklyPython
  );
  const setBlocklyXML = useBlockEditorState((state) => state.setBlocklyXML);
  const saveBlock = useMRAState((state) => state.saveBlock);

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
      {!currentBlock && <div>No Block selected.</div>}
      {currentBlock && (
        <div>
          Editing Block {currentBlock.name} ({currentBlock.id})
          <span
            className="p-2 border-2 border-blue-500"
            onClick={() => saveBlock(currentBlock.id, xml ?? "")}
          >
            Save
          </span>
        </div>
      )}
      <div ref={workspaceRef} className="w-full h-full"></div>
    </div>
  );
};
