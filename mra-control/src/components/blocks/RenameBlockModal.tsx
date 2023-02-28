import { Button, Label, Modal, TextInput } from "flowbite-react";
import React, { useState } from "react";
import { CodeBlock, useMRAState } from "../../state/useMRAState";

export const RenameBlockModal = ({
  open,
  onClose,
  block,
}: {
  open: boolean;
  onClose: () => void;
  block: CodeBlock;
}) => {
  const [name, setName] = useState<string>(block.name);
  const renameBlock = useMRAState((state) => state.renameBlock);
  return (
    <Modal show={open} onClose={onClose}>
      <Modal.Header>Rename Block</Modal.Header>
      <Modal.Body>
        <div>
          <div className="mb-2 block">
            <Label value="Block Name" />
          </div>
          <TextInput value={name} onChange={(e) => setName(e.target.value)} />
        </div>
      </Modal.Body>
      <Modal.Footer>
        <Button
          onClick={() => {
            renameBlock(name);
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
