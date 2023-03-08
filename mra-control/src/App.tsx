import React from "react";
import { Panel, PanelGroup, PanelResizeHandle } from "react-resizable-panels";
import { Timeline } from "./layout/Timeline";
import { BlockEditorPanel } from "./layout/BlockEditorPanel";
import { BlockCodePanel } from "./layout/BlockCodePanel";
import { BlockManagerPanel } from "./layout/BlockManagerPanel";
import { useRobartState } from "./state/useRobartState";
import { NavigationBar } from "./layout/NavigationBar";
import { SettingsModal } from "./layout/settings/SettingsModal";

function App() {
  return (
    <>
      <div className="flex h-screen w-screen flex-col gap-1">
        <NavigationBar />
        <div className="flex flex-grow">
          <PanelGroup direction="vertical">
            <Panel
              defaultSize={50}
              onResize={() => window.dispatchEvent(new Event("resize"))}
            >
              <PanelGroup direction="horizontal">
                <Panel
                  defaultSize={50}
                  onResize={() => window.dispatchEvent(new Event("resize"))}
                >
                  <BlockEditorPanel />
                </Panel>
                <PanelResizeHandle className="w-2 bg-blue-50 opacity-30 shadow-lg" />
                <Panel defaultSize={50}>
                  <BlockCodePanel />
                </Panel>
              </PanelGroup>
            </Panel>
            <PanelResizeHandle className="h-2 bg-blue-50 opacity-30 shadow-lg" />
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
    </>
  );
}

export default App;
