import { RenamableText } from "@MRAControl/components/utils/RenamableText";
import { Button } from "flowbite-react";
import React, { useState } from "react";
import { RenameBlockModal } from "../components/blocks/RenameBlockModal";
import { CodeBlock, useRobartState } from "../state/useRobartState";

export const BlockEditorHeader = () => {
    const currentBlockId = useRobartState((state) => state.editingBlockId);
    const currentBlock: CodeBlock | undefined = useRobartState(
        (state) => state.blocks[currentBlockId ?? ""]
    );
    const renameBlock = useRobartState((state) => state.renameBlock);

    if (!currentBlock) return <div className="m-2">No Block selected.</div>;

    return (
        <div className="m-2 flex items-center gap-2">
            <RenamableText
                text={currentBlock.name}
                className={
                    "flex h-min w-fit min-w-[10px] gap-4 rounded-md border-2 border-white p-2 text-lg font-bold transition-[width] duration-150 hover:border-black"
                }
                updateText={(newText) => {
                    newText !== ""
                        ? renameBlock(newText)
                        : renameBlock("New Block");
                }}
            />
            {/* <div className="flex text-lg font-bold">{currentBlock.name}</div>
      <div className="flex flex-grow justify-end gap-5">
        <Button
          className="border-1 flex border-green-500"
          color="gray"
          onClick={() => setRenameModalOpen(true)}
        >
          Rename
        </Button>
        <RenameBlockModal
          open={renameModalOpen}
          block={currentBlock}
          onClose={() => setRenameModalOpen(false)}
        />
      </div> */}
        </div>
    );
};
