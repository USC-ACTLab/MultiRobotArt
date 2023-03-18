import { faArrowsRotate } from "@fortawesome/free-solid-svg-icons";
import { IconButton } from "@MRAControl/components/buttons/IconButton";
import { Label, TextInput } from "flowbite-react";

export const StartingPositionEditor = () => {
    return (
        <div>
            <h3 className="ml-3 mt-3 text-lg font-extrabold">
                Starting Position:
            </h3>
            <div className="flex gap-2">
                <form>
                    <div className="flex flex-col items-center">
                        <TextInput id="x-coordinate" />
                        <Label htmlFor="x-coordinate">x</Label>
                    </div>

                    <div className="flex flex-col items-center">
                        <TextInput id="y-coordinate" />
                        <Label htmlFor="y-coordinate">y</Label>
                    </div>

                    <div className="flex flex-col items-center">
                        <TextInput id="z-coordinate" />
                        <Label htmlFor="z-coordinate">z</Label>
                    </div>

                    <input className="hidden" type="submit" />
                </form>
                <IconButton icon={faArrowsRotate} text="Update Position" />
            </div>
        </div>
    );
};
