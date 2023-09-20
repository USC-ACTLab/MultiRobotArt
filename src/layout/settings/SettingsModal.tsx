import { faSync } from '@fortawesome/free-solid-svg-icons';
import { Button, Label, Modal, Tabs, TabsRef, TextInput } from 'flowbite-react';
import React, { useRef, useState } from 'react';

import { CancelButton } from '../../components/buttons/CancelButton';
import { IconButton } from '../../components/buttons/IconButton';
import { ConfirmationModal } from '../../components/modal/ConfirmationModal';
import { useRobartState } from '../../state/useRobartState';
import { useUIState } from '../../state/useUIState';
import { CurveEditorModal } from '../curveEditor/CurveEditorModal';

export const SettingsModal = () => {
  const settingsModalOpen = useUIState((state) => state.settingsModalOpen);
  const toggleSettingsModal = useUIState((state) => state.toggleSettingsModal);
  const resetProject = useRobartState((state) => state.resetProject);
  const projectName = useRobartState((state) => state.projectName);
  const setProjectName = useRobartState((state) => state.setProjectName);
  const toggleCurveEditor = useUIState((state) => state.toggleCurveEditor);

  const [confirmOpen, setConfirmOpen] = useState(false);

  return (
    <>
      <Modal show={settingsModalOpen} onClose={toggleSettingsModal}>
        <Modal.Header>Settings</Modal.Header>
        <Modal.Body>
          <Tabs.Group style="default">
            <Tabs.Item active title="Project">
              Profile content
              <div>
                <div className="mb-2 block">
                  <Label value="Project Name" />
                </div>
                <TextInput value={projectName} onChange={(e) => setProjectName(e.target.value)} />
              </div>
            </Tabs.Item>
            <Tabs.Item title="Blocks">Block Settings</Tabs.Item>
            <Tabs.Item title="Preferences">User Preferences</Tabs.Item>
            <Tabs.Item title="Utilities">
              <Button onClick={toggleCurveEditor}>Curve Editor</Button>
            </Tabs.Item>
          </Tabs.Group>
        </Modal.Body>
        <Modal.Footer>
          <CancelButton onClick={toggleSettingsModal} />
          <IconButton color="warning" text="Reset Project" icon={faSync} onClick={() => setConfirmOpen(true)} />
        </Modal.Footer>
      </Modal>
      <ConfirmationModal
        header="Confirm Reset Project"
        open={confirmOpen}
        onCancel={() => setConfirmOpen(false)}
        onConfirm={() => {
          resetProject();
          setConfirmOpen(false);
          toggleSettingsModal();
        }}
      >
        Are you sure?
      </ConfirmationModal>
      <CurveEditorModal />
    </>
  );
};
