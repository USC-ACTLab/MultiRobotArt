import Blockly from 'blockly';
import { pythonGenerator } from 'blockly/python';
import { javascriptGenerator } from 'blockly/javascript';
import React, { useEffect, useRef, useState } from 'react';
import { useBlocklyWorkspace } from 'react-blockly';

import { blocklyToolboxConfiguration } from '../config/BlockToolboxConfig';
import '../config/customBlocks';
import { CodeBlock, useRobartState } from '../state/useRobartState';
import { BlockEditorHeader } from './BlockEditorHeader';

export const BlockEditorPanel = () => {
  const workspaceRef = useRef<any>(null);
  const [localBlockId, setLocalBlockId] = useState<string | undefined>(undefined);
  const currentBlockId = useRobartState((state) => state.editingBlockId);

  const saveBlock = useRobartState((state) => state.saveBlock);

  const { workspace, xml } = useBlocklyWorkspace({
    toolboxConfiguration: blocklyToolboxConfiguration,
    initialXml: '',
    workspaceConfiguration: {
      grid: {
        spacing: 20,
        length: 3,
        colour: '#ccc',
        snap: true,
      },
    },
    onWorkspaceChange: (workspace) => {
      const python = pythonGenerator.workspaceToCode(workspace);
      const javaScript = javascriptGenerator.workspaceToCode(workspace);
      if (localBlockId && xml) saveBlock(localBlockId, { xml, python, javaScript });
    },
    ref: workspaceRef,
  });

  useEffect(() => {
    window.dispatchEvent(new Event('resize'));

    setLocalBlockId(currentBlockId);
    if (currentBlockId) {
      workspace?.setVisible(true);
      const currentBlock: CodeBlock = useRobartState.getState().blocks[currentBlockId];
      if (currentBlock.xml && workspace) {
        var xmlDom = Blockly.Xml.textToDom(currentBlock.xml);
        Blockly.Xml.clearWorkspaceAndLoadFromXml(xmlDom, workspace);
      } else if (!currentBlock.xml && workspace) {
        workspace.clear();
      }
    } else if (workspace) {
      workspace.setVisible(false);
      workspace.clear();
    }
  }, [currentBlockId]);

  return (
    <div className="flex h-full w-full flex-col">
      <BlockEditorHeader />
      <div ref={workspaceRef} className="w-full flex-grow"></div>
    </div>
  );
};
