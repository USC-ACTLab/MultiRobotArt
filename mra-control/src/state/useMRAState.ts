import { create } from "zustand";
import uuid from "react-uuid";
import { useBlockEditorState } from "./useBlockEditorState";

export interface CodeBlock {
  id: string;
  xml: string;
  name: string;
}

export interface TimelineItem {
  blockId: string;
  /**
   * Start time of the block, in seconds from the start of the program.
   */
  startTime: number;
}

export interface TimelineLane {
  name: string;
  items: TimelineItem[];
}

export interface TimelineState {
  lanes: TimelineLane[];
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
  saveBlock: (blockId: string, xml: string) => void;
  removeBlock: (blockId: string) => void;
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
    lanes: [],
  },
  editingBlock: undefined,
  loadFile: (file) => {},
  saveToFile: () => {},
  saveBlock: (blockId: string, xml: string) => {
    set({
      definedBlocks: get().definedBlocks.map((b) => {
        if (b.id == blockId) {
          b.xml = xml;
        }
        return b;
      }),
    });
  },
  removeBlock: (id) => {
    set({ definedBlocks: get().definedBlocks.filter((a) => a.id !== id) });
    // TODO: Also remove all references to this block on the timeline
  },
  createBlock: (name) => {
    const block: CodeBlock = {
      id: uuid(),
      name: name,
      xml: "",
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
