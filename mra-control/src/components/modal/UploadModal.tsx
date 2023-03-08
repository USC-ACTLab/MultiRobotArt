import { Button, Label, Modal, TextInput } from "flowbite-react";
import React, { useCallback, useState } from "react";
import { useDropzone } from "react-dropzone";

export const UploadFileModal = ({
  open,
  onClose,
  onFileUpload,
  header,
}: {
  open: boolean;
  onClose: () => void;
  onFileUpload: (file: string) => void;
  header: string;
}) => {
  const onDrop = useCallback((acceptedFiles: File[]) => {
    acceptedFiles.forEach((file: File) => {
      const reader = new FileReader();

      reader.onabort = () => console.log("file reading was aborted!");
      reader.onerror = () => console.log("file reading failed!");
      reader.onload = () => {
        onFileUpload(reader.result as string);
        onClose();
      };

      reader.readAsBinaryString(file);
    });
  }, []);

  const { getRootProps, getInputProps, isDragActive, inputRef } = useDropzone({
    onDrop: (a) => onDrop(a),
    multiple: false,
    onDragEnter: (e: any) => {},
    onDragLeave: (e: any) => {},
    onDragOver: (e: any) => {},
    accept: {
      "application/json": [".robart"],
    },
  });

  return (
    <Modal show={open} onClose={onClose}>
      <Modal.Header>{header}</Modal.Header>
      <Modal.Body>
        <div
          className="border-1 flex justify-center border-gray-500 bg-gray-50 p-5 py-14 text-lg shadow-lg"
          {...getRootProps()}
        >
          <input ref={inputRef} {...getInputProps()} />
          {isDragActive ? (
            <p>Drop the files here ...</p>
          ) : (
            <p>Drag 'n' drop some files here, or click to select files</p>
          )}
        </div>
      </Modal.Body>
      <Modal.Footer>
        <Button color="gray" onClick={onClose}>
          Cancel
        </Button>
      </Modal.Footer>
    </Modal>
  );
};
