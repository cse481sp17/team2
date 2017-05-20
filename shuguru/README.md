# To start state machine
roslaunch fetch_gazebo playground.launch
roslaunch shuguru everything.launch

# To save destinations
roslaunch map_annotator set_goals.launch file_path:=$(rospack find shuguru)/data/dests.dat
roslaunch applications navigation.launch
rosrun application keyboard_teleop.py
rosrun map_annotator cli.py