import { create } from 'zustand';
import { immer } from 'zustand/middleware/immer';

export interface RobotManagerState {
  selectedRobotId: string | undefined;
  showRenameInput: boolean;
}

export interface RobotManagerActions {
  setSelectedRobotId: (id: string | undefined) => void;
  openRenameInput: () => void;
  closeRenameInput: () => void;
}

export const useRobotManager = create<RobotManagerState & RobotManagerActions>()(
  immer((set, get) => ({
    selectedRobotId: undefined,
    showRenameInput: false,
    setSelectedRobotId: (id) => {
      set({ selectedRobotId: id });
    },
    openRenameInput: () => {
      set({ showRenameInput: true });
    },
    closeRenameInput: () => {
      set({ showRenameInput: false });
    },
  })),
);
