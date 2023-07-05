import { MRAState } from '../state/useRobartState';
import { saveToFile } from './saveToFile';
import { exportToROS } from './exportToROS';

export const saveProjectToFile = (projectState: MRAState, fileName: string | undefined = undefined) => {
  const projectStateJson = JSON.stringify(projectState);
  fileName = fileName ?? projectState.projectName.replaceAll(' ', '') + '.robart';
  saveToFile(fileName, projectStateJson);
};

export const loadProjectFromFile = (fileContents: string): MRAState => {
  const projectState = JSON.parse(fileContents) as MRAState;
  // TODO: Other sanity checks to make sure it is valid. (ZOD)
  return projectState;
};

export const exportROS = (projectState: MRAState, fileName: string) => {
  exportToROS(projectState, fileName);
}