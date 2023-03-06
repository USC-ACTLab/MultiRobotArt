import { Button } from "flowbite-react";
import React from "react";
import { TimelineLane } from "../components/timeline/TimelineLane";
import { useBlockEditorState } from "../state/useBlockEditorState";
import { useMRAState } from "../state/useMRAState";

export const Timeline = () => {
  const blocks = useMRAState((state) => state.definedBlocks);
  const createBlock = useMRAState((state) => state.createBlock);
  const setEditingBlock = useMRAState((state) => state.setEditingBlock);
  const timelineState = useMRAState((state) => state.timelineState);

  return (
    <div className="flex h-full w-full flex-col gap-2">
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
      <div className="flex w-full flex-wrap gap-1 overflow-y-auto rounded bg-blue-100 p-4">
        {timelineState.lanes.map((lane, i) => (
          <TimelineLane lane={lane} key={i} />
        ))}
      </div>
    </div>
  );
};
