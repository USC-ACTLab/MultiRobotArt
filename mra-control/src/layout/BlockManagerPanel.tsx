import { Button } from 'flowbite-react';
import React from 'react';

import { useRobartState } from '../state/useRobartState';

export const BlockManagerPanel = () => {
  const blocksMap = useRobartState((state) => state.blocks);
  const blocks = Object.values(blocksMap);
  const createBlock = useRobartState((state) => state.createBlock);
  const setEditingBlock = useRobartState((state) => state.setEditingBlock);
  return (
    <div>
      <div className="flex flex-wrap gap-2 p-2">
        <Button
          className="flex !bg-bl"
          onClick={() => {
            const id = createBlock('New Block');
            setEditingBlock(id);
          }}
        >
          New
        </Button>
        {blocks.map((b) => (
          <Button key={b.id} className="flex !bg-ye" onClick={() => setEditingBlock(b.id)}>
            {b.name}
          </Button>
        ))}
      </div>
    </div>
  );
};
