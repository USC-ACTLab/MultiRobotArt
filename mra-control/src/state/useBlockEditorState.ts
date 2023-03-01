import { create } from "zustand";

export interface BlockEditorState {
  blocklyXML: string;
  blocklyPython: string;
}

export interface BlockEditorActions {
  setBlocklyXML: (xml: string) => void;
  setBlocklyPython: (code: string) => void;
}

export const useBlockEditorState = create<
  BlockEditorState & BlockEditorActions
>((set, get) => ({
  blocklyXML: "",
  blocklyPython: "",
  setBlocklyXML: (xml) => set({ blocklyXML: xml }),
  setBlocklyPython: (code) => set({ blocklyPython: code }),
}));
