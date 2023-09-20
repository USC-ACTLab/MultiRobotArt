import { Button, Label, Modal, TextInput } from 'flowbite-react';
import React, { useState } from 'react';

import { CodeBlock, TimelineGroupState, useRobartState } from '../../state/useRobartState';

export const EditGroupModal = ({ open, onClose, groupId }: { open: boolean; onClose: () => void; groupId: string }) => {
  const group = useRobartState((state) => state.timelineState.groups[groupId]);

  return (
    <Modal show={open} onClose={onClose}>
      <Modal.Header>Edit Group</Modal.Header>
      <Modal.Body>
        <div>
          <div className="mb-2 block">
            <Label value="Block Name" />
          </div>
          <TextInput value={group.name} onChange={(e) => {}} />
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
