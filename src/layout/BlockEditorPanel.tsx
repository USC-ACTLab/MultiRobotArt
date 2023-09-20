import { blocklyToolboxConfiguration } from '@MRAControl/config/BlockToolboxConfig';
import '@MRAControl/config/customBlocks';
import { useRobartState } from '@MRAControl/state/useRobartState';
import Blockly from 'blockly';
import { javascriptGenerator } from 'blockly/javascript';
import { pythonGenerator } from 'blockly/python';
import { useEffect, useRef, useState } from 'react';
import { useBlocklyWorkspace } from 'react-blockly';

import { BlockEditorHeader } from './BlockEditorHeader';

export const BlockEditorPanel = () => {
  const currentBlockId = useRobartState((state) => state.editingBlockId);
  const saveBlock = useRobartState((state) => state.saveBlock);

  const workspaceRef = useRef<HTMLDivElement>(null);
  const [localBlockId, setLocalBlockId] = useState<string>();

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
    if (!workspace) return;

    setLocalBlockId(currentBlockId);
    if (currentBlockId) {
      workspace.setVisible(true);
      const currentBlock = useRobartState.getState().blocks[currentBlockId];
      if (currentBlock.xml) {
        var xmlDom = Blockly.Xml.textToDom(currentBlock.xml);
        Blockly.Xml.clearWorkspaceAndLoadFromXml(xmlDom, workspace);
      } else if (!currentBlock.xml) {
        workspace.clear();
      }
    } else {
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
