import { MRAState } from "../state/useMRAState";
import { saveToFile } from "./saveToFile";

export const saveProjectToFile = (
  projectState: MRAState,
  fileName: string | undefined = undefined
) => {
  const projectStateJson = JSON.stringify(projectState);
  fileName = fileName ?? projectState.sessionName + ".robart";
  saveToFile(fileName, projectStateJson);
};

export const loadProjectFromFile = (fileContents: string): MRAState => {
  const projectState = JSON.parse(fileContents) as MRAState;
  // TODO: Other sanity checks to make sure it is valid.
  return projectState;
};
