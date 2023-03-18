import { create } from "zustand";

export interface UIState {
    settingsModalOpen: boolean;
    curveEditorOpen: boolean;
    simulationModalOpen: boolean;
    robotManagerModalOpen: boolean;
}

export interface UIActions {
    toggleSettingsModal: () => void;
    toggleCurveEditor: () => void;
    toggleSimulationModal: () => void;
    toggleRobotManager: () => void;
}

export type UIStoreState = UIState & UIActions;

/**
 * Store for non-persistent UI state.
 */
export const useUIState = create<UIStoreState>()((set, get) => ({
    settingsModalOpen: false,
    curveEditorOpen: false,
    simulationModalOpen: false,
    robotManagerModalOpen: false,
    toggleSettingsModal: () =>
        set({ settingsModalOpen: !get().settingsModalOpen }),
    toggleCurveEditor: () => set({ curveEditorOpen: !get().curveEditorOpen }),
    toggleSimulationModal: () =>
        set({ simulationModalOpen: !get().simulationModalOpen }),
    toggleRobotManager: () => {
        set({ robotManagerModalOpen: !get().robotManagerModalOpen });
    },
}));
