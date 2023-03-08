import { Button, Label, Modal, TextInput } from "flowbite-react";
import React, { useState } from "react";
import {
  CodeBlock,
  TimelineLaneState,
  useRobartState,
} from "../../state/useRobartState";

export const EditLaneModal = ({
  open,
  onClose,
  laneId,
}: {
  open: boolean;
  onClose: () => void;
  laneId: string;
}) => {
  const lane = useRobartState((state) => state.timelineState.lanes[laneId]);

  return (
    <Modal show={open} onClose={onClose}>
      <Modal.Header>Edit Lane</Modal.Header>
      <Modal.Body>
        <div>
          <div className="mb-2 block">
            <Label value="Block Name" />
          </div>
          <TextInput value={lane.name} onChange={(e) => {}} />
        </div>
      </Modal.Body>
      <Modal.Footer>
        <Button color="gray" onClick={onClose}>
          Cancel
        </Button>
      </Modal.Footer>
    </Modal>
  );
};
