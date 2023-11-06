# `mra-control` - Web UI for Robot Block Programming

## Project Description Page

Coming with some nice images and description soon...

## Running Locally

To run the project, you should just need to do the following:

1. Clone the repository, and `cd` into the `mra-control` directory.
2. Install NodeJS/NPM if you do not have it installed already.
3. Install the required packages using `npm i`.
4. Launch the application with `npm run dev`.

## Webapp Deployment

The webapp is currently deployed at https://robot-art-webapp.github.io/MultiRobotArt/ by the github account robot-art-webapp. To deploy, follow the following pipeline (assuming access to the deployment repo):

1. Commit and update the `deployment` branch with your local changes.
2. Tag your updating commit with a version number/date/helpful note.
3. build for deployment with `npm run deploy`
4. Open https://github.com/robot-art-webapp/MultiRobotArt/tree/gh-pages and Sync Fork
5. Verify that the changes have taken effect and no new bugs are introduced.

## Adding or Modifying a New Trajectory Block

Instructions for adding or modifying new blocks coming soon...

## Running on Real World Robots

Coming soon...

## TODOs:
The github issues contain some TODOs and are listed here in order of priority/when they will be done. This does not include bug fixes that are ongoing.

~~1. Multiple trajectories in one timeline block - compose multiple commands together~~
~~2. All robots (of one type) are added to one group by default - simplifies the `add group` process if testing simple things~~
3. Rotation and Translation block modifiers - rotate/translate a trajectory
4. Hide and add additional timeline lanes
5. Arbitrary parametric equations block
6. Nicer robot manager
7. Trajectory visualization


## Overall Design Considerations

This is the (proposed) scaffolding for the UI (control) part of MRA. This is a pretty basic React (Typescript) app that

Some notes for the folder structure of this sub-project:

1. `components` - Re-usable React components for use in various other components.
2. `config` - Static configuration (such as custom Blockly definitions, Toolbox definition, etc).
3. `layout` - Main layout of the page, including primary panes. Generally, just non-reusable components.
4. `state` - Definitions and global state management for the application (currently just `zustand` hooks).

Some other notes on things that I propose to use (and have done so in scaffolding this so far):

1.  `tailwind` - Keeps CSS styling in closest terms to raw CSS styles as possible, while also being very quick and modular. Fluent classes are
    provided, which aims to be accessible to folks who may have some basic CSS experience but not other higher-level frameworks (like boostrap).
2.  `flowbite` - Higher level tailwind framework that provides simple but nice looking base components (`Button`, `Modal`, etc).
3.  `zustand` - Simple/lightweight/accessible state management framework. Preferred to larger frameworks that require more boilerplate (such as
    `redux`). Allows for state to be shared between otherwise unrelated components.
4.  `react-blockly` - Blockly integration into `react` (really just a lightweight wrapper).