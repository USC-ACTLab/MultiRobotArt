import React from "react";
import CodeMirror from "@uiw/react-codemirror";
import { python } from "@codemirror/lang-python";
import { CodeBlock, useRobartState } from "../state/useRobartState";

export const BlockCodePanel = () => {
  const currentBlockId = useRobartState((state) => state.editingBlockId);
  const currentBlock: CodeBlock | undefined = useRobartState(
    (state) => state.blocks[currentBlockId ?? ""]
  );
  return (
    <div className="h-full w-full">
      <div className="text-lg font-bold">Code panel</div>
      <CodeMirror
        value={currentBlock?.python}
        className="h-full w-full"
        extensions={[python()]}
        readOnly={true}
      />
    </div>
  );
};
