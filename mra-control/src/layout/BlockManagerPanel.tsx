import { IconButton } from '@MRAControl/components/buttons/IconButton';
import { faCopy, faPlusCircle, faTrash } from '@fortawesome/free-solid-svg-icons';
import { Button } from 'flowbite-react';

import { useRobartState } from '../state/useRobartState';

export const BlockManagerPanel = () => {
  const blocksMap = useRobartState((state) => state.blocks);
  const blocks = Object.values(blocksMap);
  const removeBlock = useRobartState((state) => state.removeBlock);
  const createBlock = useRobartState((state) => state.createBlock);
  const copyBlock = useRobartState((state) => state.copyBlock);
  const selectedBlockID = useRobartState((state) => state.editingBlockId);
  const setEditingBlock = useRobartState((state) => state.setEditingBlock);

  return (
    <div>
      <div className="flex flex-wrap gap-2 p-2">
        <IconButton
          icon={faPlusCircle}
          text="New"
          onClick={() => {
            const id = createBlock('New Block');
            setEditingBlock(id);
          }}
        />
        <IconButton
          icon={faCopy}
          text="Copy"
          onClick={() => {
            if (selectedBlockID === undefined) return;
            const id = copyBlock(selectedBlockID);
            setEditingBlock(id);
          }}
        />
        <IconButton
          color="failure"
          icon={faTrash}
          text="Delete"
          onClick={() => {
            if (selectedBlockID === undefined) return;
            // TODO: Add a confirmation before removing the block
            removeBlock(selectedBlockID);
          }}
        />
      </div>
      <div className="flex flex-wrap gap-2 p-2">
        {blocks.map((b) => (
          <Button key={b.id} className="flex" onClick={() => setEditingBlock(b.id)} color="success">
            {b.name}
          </Button>
        ))}
      </div>
    </div>
  );
};
