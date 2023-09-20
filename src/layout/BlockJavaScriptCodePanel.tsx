import { javascript } from '@codemirror/lang-javascript';
import CodeMirror from '@uiw/react-codemirror';
import React from 'react';

import { CodeBlock, useRobartState } from '../state/useRobartState';

export const BlockJavaScriptCodePanel = () => {
  const currentBlockId = useRobartState((state) => state.editingBlockId);
  const currentBlock: CodeBlock | undefined = useRobartState((state) => state.blocks[currentBlockId ?? '']);
  return (
    <div className="h-full w-full">
      <CodeMirror value={currentBlock?.javaScript} className="h-full w-full" extensions={[javascript()]} readOnly={true} />
    </div>
  );
};
