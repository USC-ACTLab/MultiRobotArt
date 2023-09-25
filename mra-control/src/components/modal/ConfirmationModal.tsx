import {Button, Modal} from 'flowbite-react';
import * as React from 'react';

type ConfirmationModalProps = {
	open: boolean;
	onCancel: () => void;
	onConfirm: () => void;
	header: string;
};

export const ConfirmationModal = ({open, onCancel, onConfirm, header, children}: React.PropsWithChildren<ConfirmationModalProps>) => {
	return (
		<Modal show={open} onClose={onCancel}>
			<Modal.Header>{header}</Modal.Header>
			<Modal.Body>{children}</Modal.Body>
			<Modal.Footer>
				<Button color="gray" onClick={onCancel}>
          Cancel
				</Button>
				<Button color="failure" onClick={onConfirm}>
          Confirm
				</Button>
			</Modal.Footer>
		</Modal>
	);
};
