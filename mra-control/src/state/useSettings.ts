import { create } from "zustand";
import { createJSONStorage, persist } from "zustand/middleware";

export interface BlocklySettings {
  categories: string[]; // TODO: Use a stricter type from the blockly category definitions
}

export interface SettingsProperties {
  blocklySettings: BlocklySettings;
}

export interface SettingsActions {}

type SettingsState = SettingsActions & SettingsProperties;

export const useSettings = create<SettingsState>()(
  persist(
    (set, get) => ({
      blocklySettings: {
        categories: [],
      },
    }),
    {
      name: "settingsStore",
      storage: createJSONStorage(() => sessionStorage),
    }
  )
);
