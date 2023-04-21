Currently, the way to create a trajectory is to

1. First, create a curve by calling the functions in `curves.py`
2. Generate position data and saving that to a file by calling `generate_position_data` from `generate_trajectory.py`
   This process does't take too much time.
3. Load the data from the file and create a trajectory by calling `generate_trajectory_from_file`
   from `generate_trajectory.py`
   This does all the dirty work of fitting the curve to the trajectory and creating the trajectory object.
   This process takes a lot of time.
   (Alternatively, trajectory can be directly loaded by instantiating the `Trajectory` class and calling `load_csv`.)
4. `plot` in `plot_trajectory.py` can be used to plot the trajectory.
5. The trajectory can be executed.

An example is available in `crazyflie_examples/crazyflie_examples/test_blockly.py`

To expedite the process:

Find a way to directly create the trajectory data (x, y, z, yaw to their 8th derivatives) from the curve function.
This means getting trajectory data directly from position data.

To make testing easier:

Precompute some trajectory data and save them to a file.

