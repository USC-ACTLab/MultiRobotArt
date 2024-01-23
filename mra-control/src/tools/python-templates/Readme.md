1. Download and unzip zip files into a folder of your choosing (without another project in it)
2. Open a terminal in that folder location (you can right click in file explorer and open terminal or navigate to your folder on the command line)
3. Run ```python3 configure.py``` and follow the prompts, entering in the appropriate crazyflie IDs.
4. run ```ros2 launch launch.py```. RViz should come up. Confirm that it shows what you are expecting and all crazyflies connect.
5. Open a second terminal in a new tab (ctrl+shit+t), run ```python3 launch_nodes.py```