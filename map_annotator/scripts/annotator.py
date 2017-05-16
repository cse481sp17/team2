#!/usr/bin/env python

import math
import pickle
import rospy
from shuguru.srv import PutCommand, GetCommand
from std_msgs.msg import Header
from map_annotator.msg import UserAction, PoseNames
from geometry_msgs.msg import Quaternion, Pose, PoseStamped, PoseWithCovarianceStamped
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
    global marker_server

    if msg.command == 'save':
        poses[msg.name] = current_pose
        create_marker(msg.name, poses[msg.name])
    elif msg.command == 'delete':
        del poses[msg.name]
        marker_server.erase(msg.name)
        marker_server.applyChanges()
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
        poses[msg.name] = pose
        create_marker(msg.name, pose)

    publish_pose_names(poses)

def pose_callback(msg):
    global current_pose
    current_pose = msg.pose.pose

def publish_pose_names(poses):
    pose_names = PoseNames()
    pose_names.poses = poses.keys()
    pose_names_pub.publish(pose_names)

def handle_viz_input(msg):
    if (msg.event_type == InteractiveMarkerFeedback.POSE_UPDATE):
        marker_server.setPose(msg.marker_name, msg.pose);
        marker_server.applyChanges();
        poses[msg.marker_name] = msg.pose

def create_marker(name, pose):
    global marker_server
    # Create an interactive marker
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = 'map'
    int_marker.header.stamp = rospy.Time.now()
    int_marker.name = name
    int_marker_pose = pose
    int_marker_pose.position.z = 0.1
    int_marker.pose= int_marker_pose
    # Create controls for the interactive marker
    ### Box control
    arrow_marker = Marker()
    arrow_marker.type = Marker.ARROW
    arrow_marker.scale.x = 0.7
    arrow_marker.scale.y = 0.2
    arrow_marker.scale.z = 0.2
    arrow_marker.color.r = 0.5
    arrow_marker.color.g = 0.5
    arrow_marker.color.b = 0.5
    arrow_marker.color.a = 1.0
    arrow_control = InteractiveMarkerControl()
    arrow_control.always_visible = True
    arrow_control.markers.append(arrow_marker)
    ### Rotate control
    rotate_control = InteractiveMarkerControl()
    rotate_control.name = "move_rotate"
    rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE
    orien = Quaternion()
    orien.x = 0
    orien.y = 1
    orien.z = 0
    orien.w = 1
    rotate_control.orientation = orien
    # Add the control to the interactive marker
    int_marker.controls.append(arrow_control)
    int_marker.controls.append(rotate_control)
    marker_server.insert(int_marker, handle_viz_input)
    marker_server.applyChanges()


if __name__ == "__main__":

    rospy.init_node('annotator')
    rospy.on_shutdown(save_poses)
    rospy.Subscriber(USER_ACTIONS, UserAction, action_callback)
    rospy.Subscriber(AMCL_POSE, PoseWithCovarianceStamped, pose_callback)

    marker_server = InteractiveMarkerServer("marker_server")
    # Load marker poses from picke file
    load_poses()
    # Create interactive markers for the loaded poses
    for pose_name in poses:
        create_marker(pose_name, poses[pose_name])

    publish_pose_names(poses)

    # Wait for user input
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        r.sleep()
