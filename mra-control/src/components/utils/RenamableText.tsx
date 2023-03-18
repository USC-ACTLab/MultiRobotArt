import { faCheck, faTrash } from "@fortawesome/free-solid-svg-icons";
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import clsx from "clsx";
import { useEffect, useRef, useState } from "react";

interface RenamableTextProps {
    text: string;
    className?: string;
    updateText: (updatedText: string) => void;
}

export const RenamableText = ({
    text,
    updateText,
    className,
}: RenamableTextProps) => {
    const [showRenameInput, setShowRenameInput] = useState(false);
    const [inputValue, setInputValue] = useState("");
    const [width, setWidth] = useState(0);
    const span = useRef<HTMLSpanElement>(null);

    useEffect(() => {
        if (span.current === null) return;

        setWidth(span.current?.offsetWidth);
    }, [inputValue]);

    return !showRenameInput ? (
        <h2
            className={className}
            onClick={() => {
                setInputValue(text);
                setShowRenameInput(true);
            }}
        >
            {text}
        </h2>
    ) : (
        <form
            onSubmit={(e) => {
                e.preventDefault();
                updateText(inputValue);
                setShowRenameInput(false);
            }}
        >
            <span
                className={clsx(className, "absolute -z-40 opacity-0")}
                ref={span}
            >
                {inputValue}
            </span>
            <input
                className={clsx(className, "overflow-hidden")}
                value={inputValue}
                style={{ width }}
                onChange={(e) => setInputValue(e.target.value)}
                onBlur={() => setShowRenameInput(false)}
                autoFocus
            />
            <input type="submit" className="hidden" />
        </form>
    );
};
