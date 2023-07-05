/**
 * Downloads text as a file. Base on https://stackoverflow.com/questions/44656610/download-a-string-as-txt-file-in-react.
 * @param fileName The name of the file to download
 * @param contents
 */

 import JSZip from 'jszip';
 import { saveAs } from 'file-saver';




export const saveToFile = (fileName: string, contents: string) => {
  const element = document.createElement('a');
  //const file = new Blob([contents], { type: 'text/plain' });
  
  //creating a zipFile
  const zip = new JSZip();
  zip.generateAsync({ type: 'blob' })
  .then(function (content) {
    // Save the zip file
    saveAs(content, 'myZipFile.zip');
  })
  .catch(function (error) {
    console.error('Failed to create the zip file:', error);
  });

  element.href = URL.createObjectURL(zip);//file

  element.download = fileName;
  document.body.appendChild(element); // Required for this to work in FireFox
  element.click();
  document.body.removeChild(element); // Clean up after ourselves.
};
