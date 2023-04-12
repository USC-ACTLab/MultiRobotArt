import { create } from 'zustand';

export interface UIState {
  settingsModalOpen: boolean;
  curveEditorOpen: boolean;
  openSimulation: boolean;
  robotManagerModalOpen: boolean;
}

export interface UIActions {
  toggleSettingsModal: () => void;
  toggleCurveEditor: () => void;
  toggleRobotManager: () => void;
  toggleSimulation: () => void;
}

export type UIStoreState = UIState & UIActions;

/**
 * Store for non-persistent UI state.
 */
export const useUIState = create<UIStoreState>()((set, get) => ({
  settingsModalOpen: false,
  curveEditorOpen: false,
  openSimulation: false,
  robotManagerModalOpen: false,
  toggleSettingsModal: () => set({ settingsModalOpen: !get().settingsModalOpen }),
  toggleCurveEditor: () => set({ curveEditorOpen: !get().curveEditorOpen }),
  toggleRobotManager: () => {
    set({ robotManagerModalOpen: !get().robotManagerModalOpen });
  },
  toggleSimulation: () => {
    set({ openSimulation: !get().openSimulation });
  },
}));
