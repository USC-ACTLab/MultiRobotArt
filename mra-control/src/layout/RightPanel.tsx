import { useUIState } from "@MRAControl/state/useUIState";
import { BlockCodePanel } from "./BlockCodePanel";
import { SimulationPanel } from "./simulation/SimulationPanel";

export const RightPanel = () => {
    const openSimulation = useUIState((state) => state.openSimulation);

    return openSimulation ? <SimulationPanel /> : <BlockCodePanel />;
};
