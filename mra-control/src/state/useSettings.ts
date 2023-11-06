/* eslint-disable @typescript-eslint/no-unused-vars */
import {create} from 'zustand';
import {createJSONStorage, persist} from 'zustand/middleware';

export type BlocklySettings = {
	categories: string[]; // TODO: Use a stricter type from the blockly category definitions
};

export type SimulationSettings = {
	mode: 'light' | 'dark';
};

export type SettingsProperties = {
	blocklySettings: BlocklySettings;
	simulationSettings: SimulationSettings;
};

export type SettingsActions = Record<string, unknown>;

type SettingsState = SettingsActions & SettingsProperties;

export const useSettings = create<SettingsState>()(
	persist(
		(set, get) => ({
			blocklySettings: {
				categories: [],
			},
			simulationSettings: {
				mode: 'dark',
			},
		}),
		{
			name: 'settingsStore',
			storage: createJSONStorage(() => sessionStorage),
		},
	),
);
