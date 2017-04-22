#!/usr/bin/env python

import rospy
import pickle
from std_msgs.msg import Header
from map_annotator.msg import UserAction, PoseNames
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *

DEFAULT_PATH = "./poses.p"
POSE_NAMES = "/pose_names"
USER_ACTIONS = "/user_actions"
AMCL_POSE = "/amcl_pose"
MOVE_BASE = "/move_base_simple/goal"

pose_names_pub = rospy.Publisher(POSE_NAMES, PoseNames, queue_size=10, latch=True)
move_base_pub = rospy.Publisher(MOVE_BASE, PoseStamped, queue_size=10)

poses = {}            # Interactive marker poses
marker_server = None  # Interactive marker server
current_pose = None   # AMCL updates current_pose


def save_poses(save_path=DEFAULT_PATH):
    pickle.dump(poses, open(save_path, 'wb'))

def load_poses(load_path=DEFAULT_PATH):
    global poses
    try:
        poses = pickle.load(open(load_path, 'rb'))
    except: 
        poses = {}

def action_callback(msg):
    global poses
    if msg.command == 'save':
        poses[msg.name] = current_pose
        create_marker(msg.name, poses[msg.name])
    elif msg.command == 'delete':
        del poses[msg.name]
    elif msg.command == 'goto':
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        goal = PoseStamped()
        goal.header = header
        goal.pose = poses[msg.name]
        move_base_pub.publish(goal)
    elif msg.command == 'create':
        pose = Pose()
        create_marker(msg.name, pose)
    pose_names = PoseNames()
    pose_names.poses = poses.keys()
    pose_names_pub.publish(pose_names)

def pose_callback(msg):
    global current_pose
    current_pose = msg.pose.pose

def handle_viz_input(msg):
    print ("Rviz input!")

def create_marker(name, pose):
    global marker_server
    # Create an interactive marker
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = 'map'
    int_marker.header.stamp = rospy.Time.now()
    int_marker.name = name
    # Create controls for the interactive marker
    ### Box control
    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.scale.x = 0.4
    box_marker.scale.y = 0.4
    box_marker.scale.z = 0.4
    box_marker.color.r = 0.5
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 1.0
    box_marker.pose = pose
    box_control = InteractiveMarkerControl()
    box_control.always_visible = True
    box_control.markers.append(box_marker)
    ### Rotate control
    rotate_control = InteractiveMarkerControl()
    rotate_control.name = "move_x"
    rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    # Add the control to the interactive marker
    int_marker.controls.append(box_control)
    int_marker.controls.append(box_control)
    marker_server.insert(int_marker, handle_viz_input)
    marker_server.applyChanges()


if __name__ == "__main__":

    rospy.init_node('annotator')
    rospy.on_shutdown(save_poses)
    rospy.Subscriber(USER_ACTIONS, UserAction, action_callback)
    rospy.Subscriber(AMCL_POSE, PoseWithCovarianceStamped, pose_callback)

    marker_server = InteractiveMarkerServer("marker_server")
    load_poses()

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        r.sleep()
        

 
