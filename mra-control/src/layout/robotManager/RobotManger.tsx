import {RobotEditor} from './RobotEditor';
import {RobotSidebar} from './RobotSidebar';
import React from 'react';

export const RobotManager = () => {
	return (
		<div className="flex h-full">
			<RobotSidebar />
			<RobotEditor />
		</div>
	);
};
