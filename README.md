# MultiRobotArt

## Setup:
0. Clone this directory:
    ```
    git clone https://github.com/USC-ACTLab/MultiRobotArt.git
    ```
1. Initialize git submodules (get files from other repositories):
    ```
    git submodule init
    git submodule update
    ```
2. Install Node.js and npm if you don't already have it:
    Installation varies between distributions, but is not particularly hard
    <https://radixweb.com/blog/installing-npm-and-nodejs-on-windows-and-mac>

3. Install Blockly with npm:
    ```
    npm install --save blockly
    ```

## Maintenance:
1. Have the submodules been updated and need to be updated locally?
    ```
    git submodule status
    git submodule update
    ```
