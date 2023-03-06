import { create } from "zustand";
import uuid from "react-uuid";

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
  name: string;
  items: TimelineItem[];
}

export interface TimelineState {
  lanes: TimelineLaneState[];
}

export interface MRAState {
  definedBlocks: CodeBlock[];
  sessionName: string;
  timelineState: TimelineState;
  editingBlock: CodeBlock | undefined;
}

export interface MRAActions {
  loadFile: (file: string) => void;
  saveToFile: () => void;
  exportToPython: () => string;
  saveBlock: (blockId: string, name: string, xml: string) => void;
  removeBlock: (blockId: string) => void;
  /**
   * Renames the current editing block to have the current name.
   * @param name The name that you want to rename the block to.
   * @returns
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

export const useMRAState = create<MRAState & MRAActions>((set, get) => ({
  definedBlocks: [],
  sessionName: "New Session",
  timelineState: {
    lanes: [
      {
        name: "Lane 1",
        items: [],
      },
      {
        name: "Lane 2",
        items: [],
      },
      {
        name: "Lane 3",
        items: [],
      },
      {
        name: "Lane 4",
        items: [],
      },
    ],
  },
  editingBlock: undefined,
  loadFile: (file) => {},
  saveToFile: () => {},
  exportToPython: () => {
    return "";
  },
  saveBlock: (blockId: string, name: string, xml: string) => {
    set({
      definedBlocks: get().definedBlocks.map((b) => {
        if (b.id === blockId) {
          b.xml = xml;
          b.name = name;
        }
        return b;
      }),
    });
  },
  removeBlock: (id) => {
    set({ definedBlocks: get().definedBlocks.filter((a) => a.id !== id) });
    // TODO: Also remove all references to this block on the timeline
  },
  renameBlock: (name) =>
    set({ editingBlock: { ...get().editingBlock!, name } }),
  createBlock: (name) => {
    const block: CodeBlock = {
      id: uuid(),
      name: name,
      xml: "",
      python: "",
      duration: 10,
    };
    const blocks = get().definedBlocks;
    blocks.push(block);
    set({ definedBlocks: blocks });
    return block.id;
  },
  setEditingBlock: (blockId) => {
    const existingBlock = get().definedBlocks.filter((i) => i.id == blockId)[0];
    set({ editingBlock: existingBlock });
  },
}));
