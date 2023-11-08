import {RenamableText} from '@MRAControl/components/utils/RenamableText';

import {type CodeBlock, useRobartState} from '../state/useRobartState';
import React from 'react';

function* nameGenerator(): Generator<string> {
	let i = 0;
	while (true) {
		yield `New Block ${i}`;
		i += 1;
	}
}

export const names = nameGenerator();
export const BlockEditorHeader = () => {
	const currentBlockId = useRobartState((state) => state.editingBlockId);
	const currentBlock: CodeBlock | undefined = useRobartState((state) => state.blocks[currentBlockId ?? '']);
	const renameBlock = useRobartState((state) => state.renameBlock);

	if (!currentBlock) return <div className="h-full w-full text-3xl font-bold flex justify-center items-center">No Block Selected.</div>;

	return (
		<div className="m-2 flex items-center gap-2">
			<RenamableText
				text={currentBlock.name}
				className="text-lg font-bold"
				updateText={(newText: string) => {
					if (newText !== '') {
						renameBlock(newText);
					} else {
						renameBlock(names.next().value as string);
					}
				}}
			/>
		</div>
	);
};
