import React, { useEffect, useRef, useState } from "react";
import { useBlocklyWorkspace } from "react-blockly";
import { blocklyToolboxConfiguration } from "../config/BlockToolboxConfig";
import { pythonGenerator } from "blockly/python";
import { CodeBlock, useRobartState } from "../state/useRobartState";
import Blockly from "blockly";
import "../config/customBlocks";
import { BlockEditorHeader } from "./BlockEditorHeader";

export const BlockEditorPanel = () => {
  const workspaceRef = useRef<any>(null);
  const currentBlockId = useRobartState((state) => state.editingBlockId);

  const saveBlock = useRobartState((state) => state.saveBlock);

  const { workspace, xml } = useBlocklyWorkspace({
    toolboxConfiguration: blocklyToolboxConfiguration,
    initialXml: "",
    workspaceConfiguration: {
      grid: {
        spacing: 20,
        length: 3,
        colour: "#ccc",
        snap: true,
      },
    },
    onWorkspaceChange: (workspace) => {
      const python = pythonGenerator.workspaceToCode(workspace);

      console.log("Workspace change with id", currentBlockId);
      if (currentBlockId && xml) saveBlock(currentBlockId, { xml, python });
    },
    ref: workspaceRef,
  });

  useEffect(() => {
    console.log("Current block changed", currentBlockId);
    if (currentBlockId) {
      const currentBlock: CodeBlock =
        useRobartState.getState().blocks[currentBlockId];
      console.log(workspace, currentBlock);
      if (currentBlock.xml && workspace) {
        var xmlDom = Blockly.Xml.textToDom(currentBlock.xml);
        Blockly.Xml.clearWorkspaceAndLoadFromXml(xmlDom, workspace);
      } else if (!currentBlock.xml && workspace) {
        workspace.clear();
      }
    } else if (workspace) {
      workspace.clear();
    }
  }, [currentBlockId]);

  return (
    <div className="h-full w-full">
      <BlockEditorHeader />
      {currentBlockId && (
        <div ref={workspaceRef} className="h-full w-full"></div>
      )}
    </div>
  );
};
