import { Button } from "flowbite-react";
import React, { useState } from "react";
import { RenameBlockModal } from "../components/blocks/RenameBlockModal";
import { CodeBlock, useRobartState } from "../state/useMRAState";

export const BlockEditorHeader = () => {
  const [renameModalOpen, setRenameModalOpen] = useState<boolean>(false);
  const currentBlockId = useRobartState((state) => state.editingBlockId);
  const currentBlock: CodeBlock | undefined = useRobartState(
    (state) => state.blocks[currentBlockId ?? ""]
  );

  if (!currentBlock) return <div className="m-2">No Block selected.</div>;

  return (
    <div className="m-2 flex items-center gap-2">
      <div className="flex text-lg font-bold">{currentBlock.name}</div>
      <div className="flex flex-grow justify-end gap-5">
        <Button
          className="border-1 flex border-green-500"
          color="danger"
          onClick={() => setRenameModalOpen(true)}
        >
          Rename
        </Button>
        <RenameBlockModal
          open={renameModalOpen}
          block={currentBlock}
          onClose={() => setRenameModalOpen(false)}
        />
      </div>
    </div>
  );
};
