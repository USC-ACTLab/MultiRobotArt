import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import { faGear } from "@fortawesome/free-solid-svg-icons";

import { Button } from "flowbite-react";

export const IconButton = ({
  icon,
  text,
  onClick,
}: {
  icon: typeof faGear;
  text: string;
  onClick?: () => void;
}) => {
  return (
    <Button onClick={onClick}>
      <FontAwesomeIcon icon={icon} />
      <span className="ml-2">{text}</span>
    </Button>
  );
};
