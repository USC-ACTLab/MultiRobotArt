/* eslint-disable @typescript-eslint/no-unsafe-assignment */
/* eslint-disable @typescript-eslint/no-shadow */
/* eslint-disable @typescript-eslint/naming-convention */
/* eslint-disable @typescript-eslint/no-unused-vars */
/* eslint-disable @typescript-eslint/no-dynamic-delete */
import uuid from 'react-uuid';
import {create} from 'zustand';
import {createJSONStorage, persist, subscribeWithSelector} from 'zustand/middleware';
import {immer} from 'zustand/middleware/immer';

import {ROBART_VERSION} from '../config/Version';
import {exportROS, loadProjectFromFile, saveProjectToFile} from '../tools/projectFileConversion';
import {useSimulator} from './useSimulator';
import * as SIM from './simulatorCommands';
import {type SimulatorGroupState} from './simulatorCommands';
import {type Trajectory} from './trajectories';

const simulator = SIM;
export type CodeBlock = {
	/**
   * Code Block uuid
   */
	id: string;
	/**
   * The XML code that blockly uses to define the state of the code block.
   */
	xml: string;
	/**
   * The actual python code that a given block corresponds to.
   */
	python: string;
	/**
   * The JavaScript code that we can execute in the browser to simulate the action in browser simulation.
   */
	javaScript: string;
	/**
   * The user-defined name of the block.
   */
	name: string;
	/**
   * The user-defined duration of a block (in seconds).
   */
	duration: number;

	// block exec function
};

export type TimelineItem = {
	id: string;
	groupId: string;
	blockId: string;
	isTrajectory: boolean;
	/**
   * Start time of the block, in seconds from the start of the program.
   */
	startTime: number;
	duration: number;
};

export type TimelineGroupState = {
	id: string;
	name: string;
	items: Record<string, TimelineItem>;
	/**
   * List of robot ids assigned to this group
   */
	robots: Record<string, RobotState>;
	// Length of time line in milliseconds
	duration: number;
};

export type TimelineModes = 'ADD' | 'ERASE' | 'MOVE';
export type TimelineState = {
	groups: Record<string, TimelineGroupState>;
	mode: TimelineModes;
	scale: number;
};

export type RobotType = 'crazyflie';

export type RobotState = {
	id: string;
	name: string;
	startingPosition: [number, number, number];
	type: RobotType;
};

export type MRAState = {
	/**
   * Store of blocks created by the user.
   */
	blocks: Record<string, CodeBlock>;
	projectName: string;
	timelineState: TimelineState;
	editingBlockId: string | undefined;
	version: number;
	robots: Record<string, RobotState>;
};

export type TimelineActions = {
	/**
   * Saves a given group of the timeline. Assumes that the group already exists.
   * @param groupId The Id of the group to update.
   * @param group A partial containing properties of the group we want to save.
   */
	saveGroup: (groupId: string, group: Partial<TimelineGroupState>) => void;
	/**
   * Creates a new group in the timeline and returns its unique ID.
   * @param name The name of the group to create.
   * @returns The Id of the newly created timeline group.
   */
	createGroup: (name: string) => string;
	/**
   * Adds a block to timeline and returns its unique ID.
   * @param groupId The id of the group to add the block to
   * @param blockId The id of the block to add to the timeline group
   * @param startTime The start time of the block's execution
   */
	addBlockToTimeline: (groupId: string, blockId: string, startTime: number, isTrajectory: boolean) => void;
	removeTimelineItem: (groupId: string, itemId: string) => void;

	addRobotToGroup: (groupId: string, robotId: string) => void;
	removeRobotFromGroup: (groupId: string, robotId: string) => void;
	updateBlockInTimeline: (groupId: string, itemId: string, startTime: number) => void;
	setTimelineMode: (mode: TimelineModes) => void;
};

export type BlockActions = {
	/**
   * Saves changes to a given block. Assumes that the block is already present.
   * @param blockId The Id of the block that we want to save.
   * @param block A partial containing the fields of the block that we want to update.
   */
	saveBlock: (blockId: string, block: Partial<CodeBlock>) => void;
	/**
   * Removes a block
   * @param blockId the id of the block to remove
   */
	removeBlock: (blockId: string) => void;
	/**
   * Renames the current editing block to have the current name.
   * @param name The name that you want to rename the block to.
   */
	renameBlock: (name: string) => void;
	/**
   * Creates a new block, and returns its ID.
   * @param blockName The name of the block to create.
   * @returns The ID of the newly created block.
   */
	createBlock: (blockName: string) => string;
	/**
   * Copies a block, and returns the new ID.
   * @param blockId the id of the block to copy.
   * @returns The ID of the newly created block.
   */
	copyBlock: (blockId: string) => string;
	/**
   * Used to modify duration of block for placing on timeline
   * @param blockId the id of the block to set duration of
   * @param duration the duration of the block
   */
	setDuration: (blockId: string, duration: number) => void;
	/**
   * Sets the currently selected block.
   * @param blockId The ID of the block to select in the block editor for editing.
   */
	setEditingBlock: (blockId: string | undefined) => void;
};

export type RobotActions = {
	createRobot: () => string;
	saveRobot: (id: string, robot: Partial<RobotState>) => void;
	deleteRobot: (id: string) => void;
};

export type MRAGeneralActions = {
	loadProject: (fileContents: string) => void;
	saveProject: (fileName?: string) => void;
	resetProject: () => void;
	setProjectName: (projectName: string) => void;
	exportToROS: (filename: string) => void;
};

const defaultRobartState: MRAState = {
	blocks: {},
	projectName: 'New Robart Project',
	timelineState: {
		scale: 1,
		mode: 'ADD',
		groups: {
			group1: {
				id: 'group1',
				name: 'Group 1',
				items: {},
				robots: {},
				duration: 60,
			},
			group2: {
				id: 'group2',
				name: 'Group 2',
				items: {},
				robots: {},
				duration: 60,
			},
			group3: {
				id: 'group3',
				name: 'Group 3',
				items: {},
				robots: {},
				duration: 60,
			},
			group4: {
				id: 'group4',
				name: 'Group 4',
				items: {},
				robots: {},
				duration: 60,
			},
			group5: {
				id: 'group5',
				name: 'Group 5',
				items: {},
				robots: {},
				duration: 60,
			},
		},
	},
	editingBlockId: undefined,
	version: ROBART_VERSION,
	robots: {},
};

type MRAActions = MRAGeneralActions & TimelineActions & BlockActions & RobotActions;

type MRACompleteState = MRAState & MRAActions;

export const useRobartState = create<MRAState & MRAActions>()(
	immer(
		subscribeWithSelector(
			persist(
				(set, get): MRACompleteState => ({
					...defaultRobartState,
					loadProject: (file) => {
						const newState = loadProjectFromFile(file);
						set(newState);
					},
					saveProject: (fileName: string | undefined) => {
						const state: MRAState = {
							blocks: get().blocks,
							editingBlockId: undefined,
							projectName: get().projectName,
							timelineState: get().timelineState,
							version: ROBART_VERSION,
							robots: get().robots,
						};
						saveProjectToFile(state, fileName);
					},
					exportToROS: (fileName: string) => {
						const state: MRAState = {
							blocks: get().blocks,
							editingBlockId: undefined,
							projectName: get().projectName,
							timelineState: get().timelineState,
							version: ROBART_VERSION,
							robots: get().robots,
						};
						exportROS(state, fileName);
					},
					resetProject: () => {
						set(defaultRobartState);
					},
					setProjectName: (name) => {
						set({projectName: name}); 
					},
					saveGroup: (groupId, group) => {},
					createGroup: (name) => {
						const id = uuid();
						const group: TimelineGroupState = {
							id,
							name,
							items: {},
							robots: {},
							duration: 60,
						};
						set((state) => {
							state.timelineState.groups[id] = group;
						});
						return id;
					},
					addBlockToTimeline: (groupId: string, blockId: string, startTime: number, isTrajectory: boolean) => {
						// TODO: Check if this causes unintended consequences...
						// let simState = useSimulator.getState();
						// simState.executeSimulation(0, startTime);
						// while (simState.time < startTime) {
						// 	console.log(simState.time);
						// 	simState.step();
						// }

						const groupState: SimulatorGroupState = {
							robotIDs: Object.keys(get().timelineState.groups[groupId].robots),
						};
						const state: MRAState = {
							blocks: get().blocks,
							editingBlockId: undefined,
							projectName: get().projectName,
							timelineState: get().timelineState,
							version: ROBART_VERSION,
							robots: get().robots,
						};
 
						// Sort by start time, eval all blocks in order to accumulate duration
						// var items_list = Object.keys(items). map(function(key){
						//   return [key, items[key].startTime];
						// });
						// items_list.sort(function(a, b) {
						//   return (b[1] as number) - (a[1] as number);
						// });

						// for(var i = 0; i < items_list.length; i++){
						//   const timeLineItem = items_list[i][0];
						//   const block = get().blocks[items[timeLineItem].blockId];
						//   if (items[timeLineItem].startTime > startTime){
						//     break;
						//   }
						//   eval(block.javaScript);
						// }
						var duration = 0;
						console.log(groupState); // Do not Remove!
						console.log(simulator.dummy()); //Compiler will auto remove unused variables...
						
						// This only kind of works, doesn't work for velo commands because init position will be wrong.
						// TODO: Run all blocks up until this point in the timeline to get position

						// Loop through every line in the given block, accumulate duration
						let lines = get().blocks[blockId].javaScript.split('\n'); // Need to return a Record<robotId, Trajectory>...
						lines.forEach((line) => {
							if (line.length !== 0) {
								let [dur, trajectoryRecord]: [number, Record<string, Trajectory>] = eval(line); // Return a Trajectory lambda function? Only compute actual trajectory when ready?
								duration += dur;
							}
						});
						// eval(get().blocks[blockId].javaScript); // TODO: Totally safe, no security flaws whatsoever.
						//const currBlock = get().blocks[blockId]
						//const execute = simulator.dummy //getSimCommand(currBlock)
						//duration = execute(currBlock, groupState)

						duration = Math.max(0.1, duration);
						const newItem = {
							id: uuid(),
							groupId,
							blockId,
							startTime,
							isTrajectory,
							duration,
						};
            
						const oldItems = {...get().timelineState.groups[groupId].items};
						oldItems[newItem.id] = newItem;

						set((state) => {
							state.timelineState.groups[groupId].items = oldItems;
						});
					},
					updateBlockInTimeline: (groupId, itemId, startTime) => {
						const newItem = {
							...get().timelineState.groups[groupId].items[itemId],
							startTime,
						};
						const oldItems = {...get().timelineState.groups[groupId].items};
						oldItems[newItem.id] = newItem;

						set((state) => {
							state.timelineState.groups[groupId].items = oldItems;
						});
					},
					saveBlock: (blockId: string, block: Partial<CodeBlock>) => {
						if (get().blocks[blockId] === undefined) return;

						set((state) => {
							state.blocks[blockId] = {
								...state.blocks[blockId],
								...block,
							};
						});
					},
					removeBlock: (id) => {
						// Delete the block from the list of blocks
						const newBlocks = Object.fromEntries(Object.entries(get().blocks).filter(([key, _]) => key !== id));

						// Remove all references to the block in the timeline
						const itemsToRemove = Object.values(get().timelineState.groups).map((group) =>
							Object.values(group.items).filter((item) => item.blockId === id),
						);

						itemsToRemove.flat().forEach((item) => {
							get().removeTimelineItem(item.groupId, item.id); 
						});

						// Update the selected item
						const selectedBlockId = Object.values(newBlocks).at(-1)?.id;

						set({blocks: newBlocks, editingBlockId: selectedBlockId});
					},
					renameBlock: (name) => {
						set((state) => {
							state.blocks[state.editingBlockId!].name = name;
						}); 
					},
					createBlock: (name) => {
						const block: CodeBlock = {
							id: uuid(),
							name: name,
							xml: '',
							python: '',
							javaScript: '',
							duration: 3,
						};
						set((state) => {
							state.blocks[block.id] = block;
						});
						return block.id;
					},
					copyBlock: (blockId) => {
						const newBlock: CodeBlock = {
							...get().blocks[blockId],
							id: uuid(),
							name: `Copy of ${get().blocks[blockId].name}`,
						};

						set((state) => {
							state.blocks[newBlock.id] = newBlock;
						});

						return newBlock.id;
					},
					setDuration: (blockId, duration) => {
						set((state) => {
							state.blocks[blockId].duration = duration;
						});
					},
					setEditingBlock: (blockId) => {
						const oldEditingBlockId = get().editingBlockId;
						set({editingBlockId: undefined});
						if (blockId != oldEditingBlockId) set({editingBlockId: blockId});
					},
					addRobotToGroup: (groupId, robotId) => {
						set((state) => {
							state.timelineState.groups[groupId].robots[robotId] = state.robots[robotId];
						});
					},
					removeRobotFromGroup: (groupId, robotId) => {
						set((state) => {
							delete state.timelineState.groups[groupId].robots[robotId];
						});
					},
					createRobot: () => {
						const id = uuid();
						const numRobots = Object.keys(get().robots).length;
						set((state) => {
							state.robots[id] = {
								id,
								name: `CF ${numRobots}`,
								type: 'crazyflie',
								startingPosition: [0, 0, 0],
							};
						});
						return id;
					},
					saveRobot: (id, robot) => {
						set((state) => {
							state.robots[id] = {
								...state.robots[id],
								...robot,
							};
						});
					},
					deleteRobot: (id) => {
						const newRobots = {...get().robots};
						delete newRobots[id];

						set({robots: newRobots});
					},
					setTimelineMode: (mode) => {
						set({timelineState: {...get().timelineState, mode}});
					},
					removeTimelineItem: (groupId, itemId) => {
						const newItems = {...get().timelineState.groups[groupId].items};
						delete newItems[itemId];

						set((state) => {
							state.timelineState.groups[groupId].items = newItems;
						});
					},
				}),
				{
					storage: createJSONStorage(() => sessionStorage),
					name: 'robartState',
				},
			),
		),
	),
);

useRobartState.subscribe(
	(state) => state.robots,
	(robots) => {
		useSimulator.getState().setRobots(robots);
	},
);
