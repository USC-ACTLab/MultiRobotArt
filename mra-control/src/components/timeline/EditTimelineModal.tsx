import { Button, Label, Modal, TextInput } from "flowbite-react";
import React, { useState } from "react";
import {
  CodeBlock,
  TimelineLaneState,
  useRobartState,
} from "../../state/useRobartState";

export const EditTimelineModal = ({
  open,
  onClose,
  lane,
}: {
  open: boolean;
  onClose: () => void;
  lane: TimelineLaneState;
}) => {
  const [name, setName] = useState<string>(lane.name);
  const saveLane = useRobartState((state) => state.saveGroup);
  return (
    <Modal show={open} onClose={onClose}>
      <Modal.Header>Edit Timeline Lane</Modal.Header>
      <Modal.Body>
        <div>
          <div className="mb-2 block">
            <Label value="Lane Name" />
          </div>
          <TextInput value={name} onChange={(e) => setName(e.target.value)} />
        </div>
      </Modal.Body>
      <Modal.Footer>
        <Button
          onClick={() => {
            saveLane(lane.id, { name });
            onClose();
          }}
        >
          Rename
        </Button>
        <Button color="gray" onClick={onClose}>
          Cancel
        </Button>
      </Modal.Footer>
    </Modal>
  );
};
