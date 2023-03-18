import { Modal } from "flowbite-react";
import { useUIState } from "../../state/useUIState";
import { Canvas } from "@react-three/fiber";
import { Simulation } from "./Simulation";
import { useSimulator } from "../../state/useSimulator";
import { useEffect } from "react";
import { useRobartState } from "../../state/useRobartState";

export const CurveEditorModal = () => {
    const simulationModalOpen = useUIState(
        (state) => state.simulationModalOpen
    );
    const toggleSimulationModal = useUIState(
        (state) => state.toggleSimulationModal
    );
    const setRobots = useSimulator((state) => state.setRobots);
    const halt = useSimulator((state) => state.halt);
    const robots = useRobartState((state) => state.robots);

    useEffect(() => {
        if (simulationModalOpen) {
            halt();
            setRobots(robots);
        }
    }, [simulationModalOpen]);

    return (
        <Modal
            show={simulationModalOpen}
            onClose={toggleSimulationModal}
            className="!w-full"
            size="w-full h-5/6"
        >
            <Modal.Header>Robot Simulation</Modal.Header>
            <Modal.Body className="h-[80vh]">
                <Canvas>
                    <Simulation />
                </Canvas>
            </Modal.Body>
            <Modal.Footer></Modal.Footer>
        </Modal>
    );
};
