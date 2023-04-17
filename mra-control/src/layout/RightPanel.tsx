import { useUIState } from '@MRAControl/state/useUIState';
import { BlockPythonCodePanel } from './BlockPythonCodePanel';
import { SimulationPanel } from './simulation/SimulationPanel';
import { Button, Tabs } from 'flowbite-react';
import { BlockJavaScriptCodePanel } from './BlockJavaScriptCodePanel';
import { useRef, useState } from 'react';

type TabName = 'simulation' | 'python' | 'javascript';
export const RightPanel = () => {
  const [selectedTab, setSelectedTab] = useState<TabName>("simulation");
  return  <>
    <div className='flex flex-col gap-2 h-full'>
      <div>
      <Button.Group>
        <Button color={selectedTab == 'simulation' ? 'blue' : 'gray'} onClick={() => setSelectedTab('simulation')}>
          Simulation
        </Button>
        <Button color={selectedTab == 'python' ? 'blue' : 'gray'} onClick={() => setSelectedTab('python')}>
          Python Code
        </Button>
        <Button color={selectedTab == 'javascript' ? 'blue' : 'gray'} onClick={() => setSelectedTab('javascript')}>
          JavaScript Code
        </Button>
        </Button.Group>
      </div>
      <div className='flex-grow h-full'>
        {selectedTab == 'simulation' && <SimulationPanel />}
        {selectedTab == 'python' && <BlockPythonCodePanel />}
        {selectedTab == 'javascript' && <BlockJavaScriptCodePanel />}
      </div>
    </div>
</>
};
