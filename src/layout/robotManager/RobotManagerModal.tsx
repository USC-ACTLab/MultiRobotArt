import { Modal } from 'flowbite-react';

import { useUIState } from '../../state/useUIState';
import { RobotManager } from './RobotManger';

export const RobotManagerModal = () => {
  const robotManagerModalOpen = useUIState((state) => state.robotManagerModalOpen);
  const toggleRobotManager = useUIState((state) => state.toggleRobotManager);
  return (
    <Modal show={robotManagerModalOpen} onClose={toggleRobotManager} className="!w-full" size="w-full h-5/6">
      <Modal.Header>Robot Manager</Modal.Header>
      <Modal.Body className="h-[80vh]">
        <RobotManager />
      </Modal.Body>
      <Modal.Footer></Modal.Footer>
    </Modal>
  );
};
