import { Button } from "flowbite-react";
import React from "react";
import { useBlockEditorState } from "../state/useBlockEditorState";
import { useMRAState } from "../state/useMRAState";

export const Timeline = () => {
  const blocks = useMRAState((state) => state.definedBlocks);
  const createBlock = useMRAState((state) => state.createBlock);
  const setEditingBlock = useMRAState((state) => state.setEditingBlock);

  return (
    <div className="flex flex-row gap-2 p-2">
      <Button
        className="flex"
        onClick={() => {
          const id = createBlock("New Block");
          setEditingBlock(id);
        }}
      >
        New
      </Button>
      {blocks.map((b) => (
        <Button
          key={b.id}
          className="flex"
          onClick={() => setEditingBlock(b.id)}
          color="success"
        >
          {b.name}
        </Button>
      ))}
    </div>
  );
};
