import clsx from 'clsx';
import { useEffect, useRef, useState } from 'react';

interface RenamableTextProps {
  text: string;
  className?: string;
  updateText: (updatedText: string) => void;
}

export const RenamableText = ({ text, updateText, className }: RenamableTextProps) => {
  const [showRenameInput, setShowRenameInput] = useState(false);
  const [inputValue, setInputValue] = useState('');
  const [width, setWidth] = useState(0);
  const span = useRef<HTMLSpanElement>(null);

  useEffect(() => {
    if (span.current === null) return;

    setWidth(span.current?.offsetWidth);
  }, [inputValue]);

  const defaultClassName = 'h-min w-fit rounded-md border-2 border-white hover:border-black';

  return !showRenameInput ? (
    <h2
      className={clsx(defaultClassName, className)}
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
      <span className={clsx(defaultClassName, className, 'absolute -z-40 whitespace-pre')} ref={span}>
        {inputValue}
      </span>
      <input
        className={clsx(defaultClassName, className)}
        value={inputValue}
        style={{ width: `calc(${width}px + 1ch)` }}
        onChange={(e) => setInputValue(e.target.value)}
        onBlur={() => setShowRenameInput(false)}
        autoFocus
      />
      <input type="submit" className="hidden" />
    </form>
  );
};
