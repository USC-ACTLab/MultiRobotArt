import React from "react";
import { useBlockEditorState } from "../state/useBlockEditorState";
import { useMRAState } from "../state/useMRAState";

export const Timeline = () => {
  const blocks = useMRAState((state) => state.definedBlocks);
  const createBlock = useMRAState((state) => state.createBlock);
  const setEditingBlock = useMRAState((state) => state.setEditingBlock);

  return (
    <div className="flex flex-row">
      <div
        className="flex cursor-pointer"
        onClick={() => {
          const id = createBlock("New Block");
          setEditingBlock(id);
        }}
      >
        New
      </div>
      {blocks.map((b) => (
        <div
          key={b.id}
          className="flex m-2 border-2 border-black cursor-pointer"
          onClick={() => setEditingBlock(b.id)}
        >
          {b.name}
        </div>
      ))}
    </div>
  );
};
