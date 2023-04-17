import { RenamableText } from '@MRAControl/components/utils/RenamableText';
import { Button } from 'flowbite-react';
import React, { useState } from 'react';
import { CodeBlock, useRobartState } from '../state/useRobartState';

export const BlockEditorHeader = () => {
  const currentBlockId = useRobartState((state) => state.editingBlockId);
  const currentBlock: CodeBlock | undefined = useRobartState((state) => state.blocks[currentBlockId ?? '']);
  const renameBlock = useRobartState((state) => state.renameBlock);

  if (!currentBlock) return <div className="m-2">No Block selected.</div>;

  return (
    <div className="m-2 flex items-center gap-2">
      <RenamableText
        text={currentBlock.name}
        className="text-lg font-bold"
        updateText={(newText) => {
          newText !== '' ? renameBlock(newText) : renameBlock('New Block');
        }}
      />
    </div>
  );
};
