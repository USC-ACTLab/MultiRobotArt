import { python } from '@codemirror/lang-python';
import CodeMirror from '@uiw/react-codemirror';
import React from 'react';

import { CodeBlock, useRobartState } from '../state/useRobartState';

export const BlockPythonCodePanel = () => {
  const currentBlockId = useRobartState((state) => state.editingBlockId);
  const currentBlock: CodeBlock | undefined = useRobartState((state) => state.blocks[currentBlockId ?? '']);
  return (
    <div className="h-full w-full">
      <CodeMirror value={currentBlock?.python} className="h-full w-full" extensions={[python()]} readOnly={true} />
    </div>
  );
};
