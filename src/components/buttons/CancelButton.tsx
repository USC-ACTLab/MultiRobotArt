import { Button } from 'flowbite-react';
import React from 'react';

export const CancelButton = ({ onClick }: { onClick: () => void }) => {
  return (
    <Button color="gray" onClick={onClick}>
      Cancel
    </Button>
  );
};
