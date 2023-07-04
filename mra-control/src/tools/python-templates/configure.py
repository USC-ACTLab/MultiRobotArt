# Need to ask user which crazyflies they will use, and will tell them where to place them

import yaml
import copy
import os, sys, stat

# Load starting positions from blockly
with open('starting_positions.yaml') as f:
    starting_positions = yaml.load(f)

n_robots = len(starting_positions['positions'])

print("You will need {} crazyflies".format(n_robots))
print("Which crazyflies (by ID) will you use?")

cfs = []
for i in range(n_robots):
    cf = input("crazyflie #{}: ".format(i + 1))
    cfs.append(cf)

    
start_dict = {}
for cf, pos in zip(cfs, starting_positions['positions']):
    print("Place cf {} at location {}".format(cf, pos))
    start_dict[cf] = pos

# Make changes to local mycrazyflies.yaml
with open('crazyflies.yaml') as f:
    all_crazyflies = yaml.load(f)

my_cfs = copy.deepcopy(all_crazyflies)
for r in all_crazyflies['robots']:
    id = r.strip('cf')
    if id not in cfs:
        del my_cfs['robots'][r]
    else:
        my_cfs['robots'][r]['enabled'] = True
        my_cfs['robots'][r]['initial_position'] = start_dict[id]


with open('my_crazyflies.yaml', 'w') as f:
    yaml.dump(my_cfs, f)

# Make commands for sim and non-sim launches
with open('sim.sh', 'w') as f:
    f.write('ros2 launch launch.py config:=my_crazyflies.yaml backend:=sim')
    os.chmod('sim.sh', stat.S_IRWXU)
with open('run.sh', 'w') as f:
    f.write('ros2 launch launch.py config:=my_crazyflies.yaml')
    os.chmod('run.sh', stat.S_IRWXU)