import { faGear } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { Button } from 'flowbite-react';
import { ReactEventHandler } from 'react';

export const IconButton = ({
  icon,
  text,
  color,
  onClick,
  className,
}: {
  icon: typeof faGear;
  text: string;
  color?: 'gray' | 'warning' | 'success' | 'failure';
  className?: string;
  onClick?: ReactEventHandler;
}) => {
  return (
    <Button className={className} onClick={onClick} color={color}>
      <FontAwesomeIcon icon={icon} />
      {text !== '' ? <span className="ml-2">{text}</span> : null}
    </Button>
  );
};
