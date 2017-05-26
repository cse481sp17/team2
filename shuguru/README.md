# To start state machine
roslaunch fetch_gazebo playground.launch
roslaunch shuguru everything.launch

# To save destinations
roslaunch map_annotator set_goals.launch file_path:=$(rospack find shuguru)/data/dests.dat
roslaunch applications navigation.launch
rosrun applications keyboard_teleop.py
rosrun map_annotator cli.py

# To save poses for grabbing boxes
roslaunch applications reach_ar_markers.launch pc_bag:=$(rospack find shuguru)/data/shoeboxes.bag
rosrun applications pbd.py

# To launch frontend with given index file
python -m SimpleHTTPServer 8080 shuguru/frontend  
