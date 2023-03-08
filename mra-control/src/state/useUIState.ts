import { create } from "zustand";

export interface UIState {
  settingsModalOpen: boolean;
}

export interface UIActions {
  toggleSettingsModal: () => void;
}

export type UIStoreState = UIState & UIActions;

/**
 * Store for non-persistent UI state.
 */
export const useUIState = create<UIStoreState>()((set, get) => ({
  settingsModalOpen: false,
  toggleSettingsModal: () =>
    set({ settingsModalOpen: !get().settingsModalOpen }),
}));
