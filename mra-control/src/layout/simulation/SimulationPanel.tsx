import { Canvas } from "@react-three/fiber";
import { Simulation } from "./Simulation";

export const SimulationPanel = () => {
    return (
        <div className="h-full w-full">
            <h2 className="text-lg font-bold">Simulation</h2>
            <Canvas>
                <Simulation />
            </Canvas>
        </div>
    );
};
