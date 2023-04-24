import React from 'react';
import { Panel, PanelGroup, PanelResizeHandle } from 'react-resizable-panels';

import { BlockPythonCodePanel } from './layout/BlockPythonCodePanel';
import { BlockEditorPanel } from './layout/BlockEditorPanel';
import { BlockManagerPanel } from './layout/BlockManagerPanel';
import { NavigationBar } from './layout/NavigationBar';
import { RightPanel } from './layout/RightPanel';
import { Timeline } from './layout/Timeline';
import { SettingsModal } from './layout/settings/SettingsModal';
import { useRobartState } from './state/useRobartState';

function App() {
  return (
    <>
      <div className="flex h-screen w-screen flex-col gap-1">
        <NavigationBar />
        <div className="flex flex-grow">
          <PanelGroup direction="vertical">
            <Panel defaultSize={60} onResize={() => window.dispatchEvent(new Event('resize'))}>
              <PanelGroup direction="horizontal">
                <Panel defaultSize={50} onResize={() => window.dispatchEvent(new Event('resize'))}>
                  <BlockEditorPanel />
                </Panel>
                <PanelResizeHandle className="w-2 bg-blue-50 opacity-30 shadow-lg" />
                <Panel defaultSize={50}>
                  <RightPanel />
                </Panel>
              </PanelGroup>
            </Panel>
            <PanelResizeHandle className="h-2 bg-blue-50 opacity-30 shadow-lg" />
            <Panel defaultSize={40}>
              <PanelGroup direction="horizontal">
                <Panel defaultSize={20}>
                  <BlockManagerPanel />
                </Panel>
                <PanelResizeHandle className="w-2 bg-blue-50 opacity-30 shadow-lg" />
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
