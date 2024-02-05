1. Download and unzip zip files into a folder of your choosing (without another project in it)
2. Open a terminal in that folder location (you can right click in file explorer and open terminal or navigate to your folder on the command line)
3. You will need two terminal tabs total, you can open a new one with ctrl+shift+t.
4. Find the correct number of (working) crazyflies for your project.
5. Run ```python3 configure.py``` and follow the prompts, entering in the appropriate crazyflie IDs.
6. Place the crazyflies out in the flight space, making sure the correct crazyflies are in the correct locations and oriented correctly.
6. run ```ros2 launch launch.py``` in your first terminal. RViz should come up. Confirm that it shows what you are expecting and all crazyflies connect.
7. In a new terminal tab, run ```python3 launch_nodes.py```. This will launch the ROS nodes for each group (with robots and blocks) in your webapp.