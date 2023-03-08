import { create } from "zustand";

export interface UIState {
  settingsModalOpen: boolean;
}

export interface UIActions {
  toggleSettingsModal: () => void;
}

export type UIStoreState = UIState & UIActions;

export const useUIState = create<UIStoreState>()((set, get) => ({
  settingsModalOpen: false,
  toggleSettingsModal: () =>
    set({ settingsModalOpen: !get().settingsModalOpen }),
}));
