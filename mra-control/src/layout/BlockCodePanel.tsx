import React from "react";
import { useBlockEditorState } from "../state/useBlockEditorState";
import CodeMirror from "@uiw/react-codemirror";
import { python } from "@codemirror/lang-python";

export const BlockCodePanel = () => {
  const blocklyPython = useBlockEditorState((state) => state.blocklyPython);
  return (
    <div className="w-full h-full">
      <div className="text-lg font-bold">Code panel</div>
      <CodeMirror
        value={blocklyPython}
        className="w-full h-full"
        extensions={[python()]}
        readOnly={true}
      />
    </div>
  );
};
