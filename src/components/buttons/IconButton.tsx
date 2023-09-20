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
  type,
}: {
  icon: typeof faGear;
  text: string;
  color?: 'gray' | 'warning' | 'success' | 'failure';
  className?: string;
  onClick?: ReactEventHandler;
  type?: 'button' | 'submit' | 'reset';
}) => {
  return (
    <Button className={className} onClick={onClick} color={color} type={type}>
      <FontAwesomeIcon icon={icon} />
      {text !== '' ? <span className="ml-2">{text}</span> : null}
    </Button>
  );
};
