import { Modal, Tabs, TabsRef } from "flowbite-react";
import React, { useRef } from "react";
import { CancelButton } from "../../components/buttons/CancelButton";
import { useUIState } from "../../state/useUIState";

export const SettingsModal = () => {
  const settingsModalOpen = useUIState((state) => state.settingsModalOpen);
  const toggleSettingsModal = useUIState((state) => state.toggleSettingsModal);
  return (
    <Modal show={settingsModalOpen} onClose={toggleSettingsModal}>
      <Modal.Header>Settings</Modal.Header>
      <Modal.Body>
        <Tabs.Group style="default">
          <Tabs.Item active title="Project">
            Profile content
          </Tabs.Item>
          <Tabs.Item title="Blocks">Block Settings</Tabs.Item>
          <Tabs.Item title="Preferences">User Preferences</Tabs.Item>
        </Tabs.Group>
      </Modal.Body>
      <Modal.Footer>
        <CancelButton onClick={toggleSettingsModal} />
      </Modal.Footer>
    </Modal>
  );
};
