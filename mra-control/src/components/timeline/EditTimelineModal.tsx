import { Button, Label, Modal, TextInput } from 'flowbite-react';
import React, { useState } from 'react';

import { CodeBlock, TimelineGroupState, useRobartState } from '../../state/useRobartState';

export const EditTimelineModal = ({ open, onClose, group }: { open: boolean; onClose: () => void; group: TimelineGroupState }) => {
  const [name, setName] = useState<string>(group.name);
  const saveGroup = useRobartState((state) => state.saveGroup);
  return (
    <Modal show={open} onClose={onClose}>
      <Modal.Header>Edit Timeline Group</Modal.Header>
      <Modal.Body>
        <div>
          <div className="mb-2 block">
            <Label value="Group Name" />
          </div>
          <TextInput value={name} onChange={(e) => setName(e.target.value)} />
        </div>
      </Modal.Body>
      <Modal.Footer>
        <Button
          onClick={() => {
            saveGroup(group.id, { name });
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
