import { faSync } from '@fortawesome/free-solid-svg-icons';
import { Button, Label, Modal, Tabs, TabsRef, TextInput } from 'flowbite-react';
import ButtonGroup from 'flowbite-react/lib/esm/components/Button/ButtonGroup';
import React, { useEffect, useRef, useState } from 'react';
// import styled from 'styled-components';

import { CancelButton } from '../../components/buttons/CancelButton';
import { IconButton } from '../../components/buttons/IconButton';
import { ConfirmationModal } from '../../components/modal/ConfirmationModal';
import { useRobartState } from '../../state/useRobartState';
import { useUIState } from '../../state/useUIState';
import { CurveEditorModal } from '../curveEditor/CurveEditorModal';
import { CSSProperties } from 'react';
import { Simulation } from '../simulation/Simulation';


import ReactSwitch from "react-switch";
import { createContext } from 'react'; 
import { RobotEditor } from '../robotManager/RobotEditor';
import { useSimulator } from '@MRAControl/state/useSimulator';

//import THREE from 'three';
// import THREE from 'three';
import * as THREE from 'three';
import { PlaneGeometry } from 'three';
// import Three from 'three'; 
// import THREE from 'three';

var globalThemeStr: string = 'darkMode';



  
export const globalThemeChanger = (mode: String) =>{
  
  let globalTheme
  if (mode == 'darkMode') {
    globalThemeStr = 'darkMode';
    globalTheme = new THREE.Color('black');

  } else if (mode == 'lightMode') {
    globalThemeStr = 'lightMode';
    globalTheme = new THREE.Color('white');
  } 

  global_theme(); //update theme str
 
  //console.log('current theme:', globalThemeStr);
  
  return globalTheme;
};


// function MyComponent(str: String) {
//   const play = useSimulator((state) => state.play);
//   const [globalthemeChanger] = useState(play);

//   useEffect(() => {
//     //globalThemeChanger(str);
//     // Perform side effects or subscribe to events here

//   }, []);

export const global_theme = () =>{
  const play = useSimulator((state) => state.play);
  const pause = useSimulator((state) => state.pause);
  const halt = useSimulator((state) => state.halt);
  const step = useSimulator((state) => state.step);
  //const [globalThemeChanger] = useState(play);
  //step();
  //Simulation();
 // play();
 // var playVar = setTimeout(halt, 1230);
  //const playVar = useState(play);
 
  //halt();
  //clearTimeout(playVar);
  
  //console.log('new theme is:', globalThemeStr);
  
  return globalThemeStr;
 
}




// const lbutton = document.getElementById("lbutton");
// lbutton.addEventListener("click", globalThemeChanger('lightMode'));

export const SettingsModal = () => {
  const settingsModalOpen = useUIState((state) => state.settingsModalOpen);
  const toggleSettingsModal = useUIState((state) => state.toggleSettingsModal);
  const resetProject = useRobartState((state) => state.resetProject);
  const projectName = useRobartState((state) => state.projectName);
  const setProjectName = useRobartState((state) => state.setProjectName);
  const toggleCurveEditor = useUIState((state) => state.toggleCurveEditor);
  const robots = useSimulator((state) => state.robots);
  

  const [confirmOpen, setConfirmOpen] = useState(false);
  let globalTheme = new THREE.Color( 'skyblue' );
  let themeString: String = 'darkMode';

  

  const buttonStyle: CSSProperties = {
    color: 'white',
    backgroundColor: 'navy',
    display:'flex',
    flexDirection: 'column',
    justifyContent:'space-evenly',
    //gap: '16px',
    fontSize: '16px',
    marginBottom: '24px'
  };

  const plane = document.getElementsByTagName('Plane');
  return (
    <>
      <Modal show={settingsModalOpen} onClose={toggleSettingsModal}>
        <Modal.Header>Settings</Modal.Header>
        <Modal.Body>
          <Tabs.Group style="default">
            <Tabs.Item active title="Project">
              Profile content
              <div>
                <div className="mb-2 block">
                  <Label value="Project Name" />
                </div>
                <TextInput value={projectName} onChange={(e) => setProjectName(e.target.value)} />
              </div>
            </Tabs.Item>
            <Tabs.Item title="Blocks">Block Settings</Tabs.Item>
            <Tabs.Item title="Preferences">

        
           
              <Button id="lbutton" style={buttonStyle} onClick={() => globalThemeChanger('darkMode') }  > Dark Mode ☾ </Button>  
              <Button id="dbutton" style={buttonStyle} onClick={() =>  globalThemeChanger('lightMode')} > Light Mode 𖤓 </Button>
           {/* onClick={changeTheme}
           onClick={changeTheme} */}
            

        



              </Tabs.Item>
            <Tabs.Item title="Utilities">
              <Button onClick={toggleCurveEditor}>Curve Editor</Button>
            </Tabs.Item>
          </Tabs.Group>
        </Modal.Body>
        <Modal.Footer>
          <CancelButton onClick={toggleSettingsModal} />
          <IconButton color="warning" text="Reset Project" icon={faSync} onClick={() => setConfirmOpen(true)} />
        </Modal.Footer>
      </Modal>
      <ConfirmationModal
        header="Confirm Reset Project"
        open={confirmOpen}
        onCancel={() => setConfirmOpen(false)}
        onConfirm={() => {
          resetProject();
          setConfirmOpen(false);
          toggleSettingsModal();
        }}
      >
        Are you sure?
      </ConfirmationModal>
      <CurveEditorModal />
    </>
  );
};
