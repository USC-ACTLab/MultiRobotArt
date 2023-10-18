import {IconButton} from '@MRAControl/components/buttons/IconButton';
import {faCopy, faPlusCircle, faTrash} from '@fortawesome/free-solid-svg-icons';
import {Button} from 'flowbite-react';
import React from 'react';
import {names} from './BlockEditorHeader';

import {useRobartState} from '../state/useRobartState';

export const BlockManagerPanel = () => {
	const blocks = useRobartState((state) => Object.values(state.blocks));
	const removeBlock = useRobartState((state) => state.removeBlock);
	const createBlock = useRobartState((state) => state.createBlock);
	const copyBlock = useRobartState((state) => state.copyBlock);
	const selectedBlockId = useRobartState((state) => state.editingBlockId);
	const setEditingBlock = useRobartState((state) => state.setEditingBlock);

	return (
		<div>
			<div className="flex flex-wrap gap-2 p-2">
				<IconButton
					icon={faPlusCircle}
					text="New"
					onClick={() => {
						const id = createBlock(names.next().value as string);
						setEditingBlock(id);
					}}
				/>
				<IconButton
					icon={faCopy}
					text="Copy"
					onClick={() => {
						if (selectedBlockId === undefined) return;
						const id = copyBlock(selectedBlockId);
						setEditingBlock(id);
					}}
				/>
				<IconButton
					color="failure"
					icon={faTrash}
					text="Delete"
					onClick={() => {
						if (selectedBlockId === undefined) return;
						// TODO: Add a confirmation before removing the block
						removeBlock(selectedBlockId);
					}}
				/>
			</div>
			<div className="flex flex-wrap gap-2 p-2">
				{blocks.map((b) => (
					<Button key={b.id} className="flex" onClick={() => {
						setEditingBlock(b.id); 
					}} color="success">
						{b.name}
					</Button>
				))}
			</div>
		</div>
	);
};
