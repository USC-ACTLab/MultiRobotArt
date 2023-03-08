import React from "react";
import { Panel, PanelGroup, PanelResizeHandle } from "react-resizable-panels";
import { Timeline } from "./layout/Timeline";
import { BlockEditorPanel } from "./layout/BlockEditorPanel";
import { BlockCodePanel } from "./layout/BlockCodePanel";
import { BlockManagerPanel } from "./layout/BlockManagerPanel";
import { useRobartState } from "./state/useMRAState";
import { NavigationBar } from "./layout/NavigationBar";

function App() {
  return (
    <div className="flex h-screen w-screen flex-col gap-1">
      <NavigationBar />
      <div className="flex flex-grow">
        <PanelGroup direction="vertical">
          <Panel defaultSize={50}>
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
          <Panel defaultSize={50}>
            <PanelGroup direction="horizontal">
              <Panel defaultSize={30}>
                <BlockManagerPanel />
              </Panel>
              <Panel>
                <Timeline />
              </Panel>
            </PanelGroup>
          </Panel>
        </PanelGroup>
      </div>
    </div>
  );
}

export default App;
