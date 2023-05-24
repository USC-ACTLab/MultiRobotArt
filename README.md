# `mra-control` - Web UI for Robot Block Programming

## Instructions

To run the project, you should just need to do the following:

1. Clone the repository, and `cd` into the `mra-control` directory.
2. Install NodeJS/NPM if you do not have it installed already.
3. Install the required packages using `npm i`.
4. Launch the application with `npm run dev`.

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
