import {IconButton} from '@MRAControl/components/buttons/IconButton';
import {useRobartState} from '@MRAControl/state/useRobartState';
import {faArrowsRotate} from '@fortawesome/free-solid-svg-icons';
import clsx from 'clsx';
import {Label, TextInput} from 'flowbite-react';
import {type ReactEventHandler, useState} from 'react';
import React from 'react';

export const StartingPositionEditor = ({robotId}: {robotId: string}) => {
	const selectedRobot = useRobartState((state) => state.robots[robotId]);
	const updateRobot = useRobartState((state) => state.saveRobot);

	const [startingPosition, setStartingPosition] = useState<[string, string, string]>([
		selectedRobot.startingPosition[0].toString(),
		selectedRobot.startingPosition[1].toString(),
		selectedRobot.startingPosition[2].toString(),
	]);

	const handleSubmit: ReactEventHandler = (e) => {
		e.preventDefault();
		const x = parseFloat(startingPosition[0]);
		const y = parseFloat(startingPosition[1]);
		const z = parseFloat(startingPosition[2]);
		if (isNaN(x) || isNaN(y) || isNaN(z)) return;
		updateRobot(robotId, {startingPosition: [x, y, z]});
	};

	return (
		<div>
			<h3 className="ml-3 mt-3 text-lg font-extrabold">Starting Position:</h3>
			<div className="flex">
				<form onSubmit={handleSubmit} className="flex flex-col gap-2">
					<div className={clsx('flex flex-col items-center', isNaN(parseFloat(startingPosition[0])) ? 'border-red-500' : '')}>
						<TextInput id="x-coordinate" value={startingPosition[0]} onChange={(e) => {
							setStartingPosition(([, y, z]) => [e.target.value, y, z]); 
						}} />
						<Label htmlFor="x-coordinate" className={isNaN(parseFloat(startingPosition[0])) ? 'text-red-500' : ''}>
              x
						</Label>
					</div>

					<div className={clsx('flex flex-col items-center', isNaN(parseFloat(startingPosition[0])) ? 'text-red-500' : '')}>
						<TextInput id="y-coordinate" value={startingPosition[1]} onChange={(e) => {
							setStartingPosition(([x,, z]) => [x, e.target.value, z]); 
						}} />
						<Label htmlFor="y-coordinate" className={isNaN(parseFloat(startingPosition[1])) ? 'text-red-500' : ''}>
              y
						</Label>
					</div>

					<div className={clsx('flex flex-col items-center', isNaN(parseFloat(startingPosition[2])) ? 'border-red-500' : '')}>
						<TextInput id="z-coordinate" value={startingPosition[2]} onChange={(e) => {
							setStartingPosition(([x, y]) => [x, y, e.target.value]); 
						}} />
						<Label htmlFor="z-coordinate" className={isNaN(parseFloat(startingPosition[2])) ? 'text-red-500' : ''}>
              z
						</Label>
					</div>

					<IconButton className="w-full" icon={faArrowsRotate} text="Update Position" onClick={handleSubmit} type="submit" />
				</form>
			</div>
		</div>
	);
};
