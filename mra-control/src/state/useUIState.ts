import { create } from "zustand";

export interface UIState {
  settingsModalOpen: boolean;
  curveEditorOpen: boolean;
}

export interface UIActions {
  toggleSettingsModal: () => void;
  toggleCurveEditor: () => void;
}

export type UIStoreState = UIState & UIActions;

/**
 * Store for non-persistent UI state.
 */
export const useUIState = create<UIStoreState>()((set, get) => ({
  settingsModalOpen: false,
  curveEditorOpen: false,
  toggleSettingsModal: () =>
    set({ settingsModalOpen: !get().settingsModalOpen }),
  toggleCurveEditor: () => set({ curveEditorOpen: !get().curveEditorOpen }),
}));
