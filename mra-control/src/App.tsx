import { useState } from "react";
import React from "react";
import { Panel, PanelGroup, PanelResizeHandle } from "react-resizable-panels";
import { Timeline } from "./layout/Timeline";
import { BlockEditorPanel } from "./layout/BlockEditorPanel";
import { BlockCodePanel } from "./layout/BlockCodePanel";

function App() {
  return (
    <div className="flex w-screen h-screen">
      <PanelGroup direction="vertical">
        <Panel defaultSize={70}>
          <PanelGroup direction="horizontal">
            <Panel defaultSize={50}>
              <BlockEditorPanel />
            </Panel>
            <PanelResizeHandle className="w-1 border-4 border-black" />
            <Panel defaultSize={50}>
              <BlockCodePanel />
            </Panel>
          </PanelGroup>
        </Panel>
        <PanelResizeHandle className="h-1 border-4 border-black" />
        <Panel defaultSize={30}>
          <Timeline />
        </Panel>
      </PanelGroup>
    </div>
  );
}

export default App;
