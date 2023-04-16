import { useUIState } from '@MRAControl/state/useUIState';

import { BlockPythonCodePanel } from './BlockPythonCodePanel';
import { SimulationPanel } from './simulation/SimulationPanel';
import { Tabs } from 'flowbite-react';
import { BlockJavaScriptCodePanel } from './BlockJavaScriptCodePanel';

export const RightPanel = () => {
  const openSimulation = useUIState((state) => state.openSimulation);

  return (
    <>
      <Tabs.Group style="default">
        <Tabs.Item active title="Simulation">
          <SimulationPanel />
        </Tabs.Item>
        <Tabs.Item active title="Python Output">
          <BlockPythonCodePanel />
        </Tabs.Item>
        <Tabs.Item active title="JavaScript Output">
          <BlockJavaScriptCodePanel />
        </Tabs.Item>
      </Tabs.Group>
    </>
  );
};
