## State machine reads data/grab_box.json and data/dests.dat
## Don't forget changing the symlink accordingly (simulation vs. real robot).

# To start state machine in simulation
# Change state_machine.py to refer to simulation/dests.dat
roslaunch fetch_gazebo playground.launch
roslaunch shuguru everything_sim.launch

# To start state machine on real robot
roslaunch shuguru everything.launch
roslaunch shuguru visualization.launch


# To save destinations
roslaunch map_annotator set_goals.launch file_path:=$(rospack find shuguru)/data/dests.dat
roslaunch applications navigation.launch map_file:=$(rospack find shuguru)/maps/map.yaml

" Save as dests.dat"

# To save poses for grabbing boxes
roslaunch applications reach_ar_markers.launch pc_bag:=$(rospack find shuguru)/data/shoeboxes.bag
rosrun applications pbd.py

" Save as shoeboxes.bag"

# To launch frontend with given index file
shuguru/frontend/server.sh 

Set the websocket:
    robot  ws://astro.cs.washington.edu:9090
    local  ws://localhost:9090

# Changing Point Cloud Location
" Change the corresponding x, y, z arg under everything.launch node
 static_transform_publisher arg value"
