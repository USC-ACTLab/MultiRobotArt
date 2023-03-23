/**
 * Downloads text as a file. Base on https://stackoverflow.com/questions/44656610/download-a-string-as-txt-file-in-react.
 * @param fileName The name of the file to download
 * @param contents
 */
export const saveToFile = (fileName: string, contents: string) => {
  const element = document.createElement('a');
  const file = new Blob([contents], { type: 'text/plain' });
  element.href = URL.createObjectURL(file);
  element.download = fileName;
  document.body.appendChild(element); // Required for this to work in FireFox
  element.click();
  document.body.removeChild(element); // Clean up after ourselves.
};
