import { create } from "zustand";
import { createJSONStorage, persist } from "zustand/middleware";
import { immer } from "zustand/middleware/immer";
import uuid from "react-uuid";
import { ROBART_VERSION } from "../config/Version";
import {
  loadProjectFromFile,
  saveProjectToFile,
} from "../tools/projectFileConversion";

export interface CodeBlock {
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
   * The user-defined name of the block.
   */
  name: string;
  /**
   * The user-defined duration of a block (in seconds).
   */
  duration: number;
}

export interface TimelineItem {
  blockId: string;
  /**
   * Start time of the block, in seconds from the start of the program.
   */
  startTime: number;
}

export interface TimelineLaneState {
  id: string;
  name: string;
  items: TimelineItem[];
}

export interface TimelineState {
  lanes: Record<string, TimelineLaneState>;
  scale: number;
}

export interface MRAState {
  /**
   * Store of blocks created by the user.
   */
  blocks: Record<string, CodeBlock>;
  sessionName: string;
  timelineState: TimelineState;
  editingBlockId: string | undefined;
  version: number;
}

export interface TimelineActions {
  /**
   * Saves a given lane of the timeline. Assumes that the lane already exists.
   * @param laneId The Id of the lane to update.
   * @param lane A partial containing properties of the lane we want to save.
   */
  saveLane: (laneId: string, lane: Partial<TimelineLaneState>) => void;
  /**
   * Creates a new lane in the timeline and returns its unique ID.
   * @param name The name of the lane to create.
   * @returns The Id of the newly created timeline lane.
   */
  createLane: (name: string) => string;
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

export interface MRAGeneralActions {
  loadFile: (fileContents: string) => void;
  saveToFile: (fileName: string | undefined) => void;
  exportToPython: () => string;
}

type MRAActions = MRAGeneralActions & TimelineActions & BlockActions;

type MRACompleteState = MRAState & MRAActions;

export const useRobartState = create<MRAState & MRAActions>()(
  immer(
    persist(
      (set, get): MRACompleteState => ({
        blocks: {},
        sessionName: "New Session",
        timelineState: {
          scale: 1,
          lanes: {
            lane1: {
              id: "lane1",
              name: "Lane 1",
              items: [],
            },
          },
        },
        editingBlockId: undefined,
        version: ROBART_VERSION,
        loadFile: (file) => {
          const newState = loadProjectFromFile(file);
          set(newState);
        },
        saveToFile: (fileName: string | undefined) => {
          const state: MRAState = {
            blocks: get().blocks,
            editingBlockId: undefined,
            sessionName: get().sessionName,
            timelineState: get().timelineState,
            version: ROBART_VERSION,
          };
          saveProjectToFile(state, fileName);
        },
        exportToPython: () => {
          return "";
        },
        saveLane: (laneId, lane) => {},
        createLane: (name) => {
          const id = uuid();
          const lane: TimelineLaneState = {
            id,
            name,
            items: [],
          };
          set((state) => {
            state.timelineState.lanes[id] = lane;
          });
          return id;
        },
        saveBlock: (blockId: string, block: Partial<CodeBlock>) => {
          set((state) => {
            state.blocks[blockId] = { ...state.blocks[blockId], ...block };
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
            xml: "",
            python: "",
            duration: 10,
          };
          set((state) => {
            state.blocks[block.id] = block;
          });
          return block.id;
        },
        setEditingBlock: (blockId) => {
          set({ editingBlockId: blockId });
        },
      }),
      {
        storage: createJSONStorage(() => sessionStorage),
        name: "robartState",
      }
    )
  )
);
