import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import { faGear } from "@fortawesome/free-solid-svg-icons";

import { Button } from "flowbite-react";

export const IconButton = ({
  icon,
  text,
  color,
  onClick,
}: {
  icon: typeof faGear;
  text: string;
  color?: "gray" | "warning";
  onClick?: () => void;
}) => {
  return (
    <Button onClick={onClick} color={color}>
      <FontAwesomeIcon icon={icon} />
      <span className="ml-2">{text}</span>
    </Button>
  );
};
