import uuid from 'react-uuid';
import { create } from 'zustand';
import { createJSONStorage, persist, subscribeWithSelector } from 'zustand/middleware';
import { immer } from 'zustand/middleware/immer';

import { ROBART_VERSION } from '../config/Version';
import { loadProjectFromFile, saveProjectToFile } from '../tools/projectFileConversion';
import { useSimulator } from './useSimulator';

export interface CodeBlock {
  /**
   * Code Block uuid
   */
  id: string;
  /**
   * The XML code that blockly uses to define the state of the code block.
   */
  xml: string;
  /**
   * The actual python code that a given block corresponds to.
   */
  python: string;
  /**
   * The JavaScript code that we can execute in the browser to simulate the action in browser simulation.
   */
  javaScript: string;
  /**
   * The user-defined name of the block.
   */
  name: string;
  /**
   * The user-defined duration of a block (in seconds).
   */
  duration: number;
}

export interface TimelineItem {
  id: string;
  groupId: string;
  blockId: string;
  /**
   * Start time of the block, in seconds from the start of the program.
   */
  startTime: number;
}

export interface TimelineGroupState {
  id: string;
  name: string;
  items: Record<string, TimelineItem>;
  /**
   * List of robot ids assigned to this group
   */
  robots: Record<string, RobotState>;
}

export interface TimelineState {
  groups: Record<string, TimelineGroupState>;
  scale: number;
}

export type RobotType = 'crazyflie';

export interface RobotState {
  id: string;
  name: string;
  startingPosition: [number, number, number];
  type: RobotType;
}

export interface MRAState {
  /**
   * Store of blocks created by the user.
   */
  blocks: Record<string, CodeBlock>;
  projectName: string;
  timelineState: TimelineState;
  editingBlockId: string | undefined;
  version: number;
  robots: Record<string, RobotState>;
}

export interface TimelineActions {
  /**
   * Saves a given group of the timeline. Assumes that the group already exists.
   * @param groupId The Id of the group to update.
   * @param group A partial containing properties of the group we want to save.
   */
  saveGroup: (groupId: string, group: Partial<TimelineGroupState>) => void;
  /**
   * Creates a new group in the timeline and returns its unique ID.
   * @param name The name of the group to create.
   * @returns The Id of the newly created timeline group.
   */
  createGroup: (name: string) => string;
  /**
   * Adds a block to timeline and returns its unique ID.
   * @param groupId The id of the group to add the block to
   * @param blockId The id of the block to add to the timeline group
   * @param startTime The start time of the block's execution
   */
  addBlockToTimeline: (groupId: string, blockId: string, startTime: number) => void;

  addRobotToGroup: (groupId: string, robotId: string) => void;
  removeRobotFromGroup: (groupId: string, robotId: string) => void;
  updateBlockInTimeline: (groupId: string, itemId: string, startTime: number) => void;
}

export interface BlockActions {
  /**
   * Saves changes to a given block. Assumes that the block is already present.
   * @param blockId The Id of the block that we want to save.
   * @param block A partial containing the fields of the block that we want to update.
   */
  saveBlock: (blockId: string, block: Partial<CodeBlock>) => void;
  removeBlock: (blockId: string) => void;
  /**
   * Renames the current editing block to have the current name.
   * @param name The name that you want to rename the block to.
   */
  renameBlock: (name: string) => void;
  /**
   * Creates a new block, and returns its ID.
   * @param blockName The name of the block to create.
   * @returns The ID of the newly created block.
   */
  createBlock: (blockName: string) => string;
  /**
   * Sets the currently selected block.
   * @param blockId The ID of the block to select in the block editor for editing.
   */
  setEditingBlock: (blockId: string | undefined) => void;
}

export interface RobotActions {
  createRobot: () => string;
  saveRobot: (id: string, robot: Partial<RobotState>) => void;
  deleteRobot: (id: string) => void;
}

export interface MRAGeneralActions {
  loadProject: (fileContents: string) => void;
  saveProject: (fileName?: string) => void;
  resetProject: () => void;
  setProjectName: (projectName: string) => void;
  exportToPython: () => string;
}

const defaultRobartState: MRAState = {
  blocks: {},
  projectName: 'New Robart Project',
  timelineState: {
    scale: 1,
    groups: {
      group1: {
        id: 'group1',
        name: 'Group 1',
        items: {},
        robots: {},
      },
      group2: {
        id: 'group2',
        name: 'Group 2',
        items: {},
        robots: {},
      },
      group3: {
        id: 'group3',
        name: 'Group 3',
        items: {},
        robots: {},
      },
      group4: {
        id: 'group4',
        name: 'Group 4',
        items: {},
        robots: {},
      },
      group5: {
        id: 'group5',
        name: 'Group 5',
        items: {},
        robots: {},
      },
      group6: {
        id: 'group6',
        name: 'Group 6',
        items: {},
        robots: {},
      },
      group7: {
        id: 'group7',
        name: 'Group 7',
        items: {},
        robots: {},
      },
      group8: {
        id: 'group8',
        name: 'Group 8',
        items: {},
        robots: {},
      },
      group9: {
        id: 'group9',
        name: 'Group 9',
        items: {},
        robots: {},
      },
    },
  },
  editingBlockId: undefined,
  version: ROBART_VERSION,
  robots: {},
};

type MRAActions = MRAGeneralActions & TimelineActions & BlockActions & RobotActions;

type MRACompleteState = MRAState & MRAActions;

export const useRobartState = create<MRAState & MRAActions>()(
  immer(
    subscribeWithSelector(
      persist(
        (set, get): MRACompleteState => ({
          ...defaultRobartState,
          loadProject: (file) => {
            const newState = loadProjectFromFile(file);
            set(newState);
          },
          saveProject: (fileName: string | undefined) => {
            const state: MRAState = {
              blocks: get().blocks,
              editingBlockId: undefined,
              projectName: get().projectName,
              timelineState: get().timelineState,
              version: ROBART_VERSION,
              robots: get().robots,
            };
            saveProjectToFile(state, fileName);
          },
          resetProject: () => {
            set(defaultRobartState);
          },
          setProjectName: (name) => set({ projectName: name }),
          exportToPython: () => {
            return '';
          },
          saveGroup: (groupId, group) => {},
          createGroup: (name) => {
            const id = uuid();
            const group: TimelineGroupState = {
              id,
              name,
              items: {},
              robots: {},
            };
            set((state) => {
              state.timelineState.groups[id] = group;
            });
            return id;
          },
          addBlockToTimeline: (groupId, blockId, startTime) => {
            const newItem = {
              id: uuid(),
              groupId,
              blockId,
              startTime,
            };

            const oldItems = { ...get().timelineState.groups[groupId].items };
            oldItems[newItem.id] = newItem;

            set((state) => {
              state.timelineState.groups[groupId].items = oldItems;
            });
          },
          updateBlockInTimeline: (groupId, itemId, startTime) => {
            const newItem = {
              ...get().timelineState.groups[groupId].items[itemId],
              startTime,
            };
            const oldItems = { ...get().timelineState.groups[groupId].items };
            oldItems[newItem.id] = newItem;

            set((state) => {
              state.timelineState.groups[groupId].items = oldItems;
            });
          },
          saveBlock: (blockId: string, block: Partial<CodeBlock>) => {
            set((state) => {
              state.blocks[blockId] = {
                ...state.blocks[blockId],
                ...block,
              };
            });
          },
          removeBlock: (id) => {
            set((state) => delete state.blocks[id]);
            // TODO: Also remove all references to this block on the timeline
          },
          renameBlock: (name) =>
            set((state) => {
              state.blocks[state.editingBlockId!].name = name;
            }),
          createBlock: (name) => {
            const block: CodeBlock = {
              id: uuid(),
              name: name,
              xml: '',
              python: '',
              javaScript: '',
              duration: 10,
            };
            set((state) => {
              state.blocks[block.id] = block;
            });
            return block.id;
          },
          setEditingBlock: (blockId) => {
            const oldEditingBlockId = get().editingBlockId;
            set({ editingBlockId: undefined });
            if (blockId != oldEditingBlockId) set({ editingBlockId: blockId });
          },
          addRobotToGroup: (groupId, robotId) => {
            set((state) => {
              state.timelineState.groups[groupId].robots[robotId] = state.robots[robotId];
            });
          },
          removeRobotFromGroup: (groupId, robotId) => {
            set((state) => {
              delete state.timelineState.groups[groupId].robots[robotId];
            });
          },
          createRobot: () => {
            const id = uuid();
            const numRobots = Object.keys(get().robots).length;
            set((state) => {
              state.robots[id] = {
                id,
                name: `CF ${numRobots}`,
                type: 'crazyflie',
                startingPosition: [0, 0, 0],
              };
            });
            return id;
          },
          saveRobot: (id, robot) => {
            set((state) => {
              state.robots[id] = {
                ...state.robots[id],
                ...robot,
              };
            });
          },
          deleteRobot: (id) => {
            const newRobots = { ...get().robots };
            delete newRobots[id];

            set({ robots: newRobots });
          },
        }),
        {
          storage: createJSONStorage(() => sessionStorage),
          name: 'robartState',
        },
      ),
    ),
  ),
);

useRobartState.subscribe(
  (state) => state.robots,
  (robots) => {
    useSimulator.getState().setRobots(robots);
  },
);
