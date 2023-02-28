import React, { useRef } from "react";
import { BlocklyWorkspace } from "react-blockly";
import { blocklyToolboxConfiguration } from "../config/BlockToolboxConfig";
import { useBlockEditorState } from "../state/useBlockEditorState";
import { pythonGenerator } from "blockly/python";

export const BlockEditorPanel = () => {
  const blocklyXML = useBlockEditorState((state) => state.blocklyXML);
  const setBlocklyPython = useBlockEditorState(
    (state) => state.setBlocklyPython
  );
  const setBlocklyXML = useBlockEditorState((state) => state.setBlocklyXML);

  return (
    <div className="w-full h-full">
      <BlocklyWorkspace
        className="w-full h-full" // you can use whatever classes are appropriate for your app's CSS
        toolboxConfiguration={blocklyToolboxConfiguration} // this must be a JSON toolbox definition
        initialXml={blocklyXML}
        workspaceConfiguration={{
          grid: {
            spacing: 20,
            length: 3,
            colour: "#ccc",
            snap: true,
          },
        }}
        onXmlChange={(xml) => console.log(xml)}
        onWorkspaceChange={(workspace) => {
          const code = pythonGenerator.workspaceToCode(workspace);
          setBlocklyPython(code);
        }}
      />
    </div>
  );
};
