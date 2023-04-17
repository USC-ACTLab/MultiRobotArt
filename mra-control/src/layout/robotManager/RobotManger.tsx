import { RobotEditor } from './RobotEditor';
import { RobotSidebar } from './RobotSidebar';

export const RobotManager = () => {
  return (
    <div className="flex h-full">
      <RobotSidebar />
      <RobotEditor />
    </div>
  );
};
