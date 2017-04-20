#!/usr/bin/env python

import rospy
import pickle
from map_annotator.msg import UserAction, PoseNames

DEFAULT_PATH = "./poses.p"
POSE_NAMES = "/pose_names"
USER_ACTIONS = "/user_actions"

poses = {}

def save_poses(save_path=DEFAULT_PATH):
    global poses
    pickle.dump(poses, open(save_path, 'wb'))


def load_poses(load_path=DEFAULT_PATH):
    global poses
    try:
        poses = pickle.load(open(load_path, 'rb'))
    except: 
        poses = {}


def action_callback():
    print ("Do something")


if __name__ == "__main__":
    rospy.init_node('annotator')
    rospy.on_shutdown(save_poses)

    global poses
    load_poses()

    pub = rospy.Publisher(POSE_NAMES, PoseNames, queue_size=10, latch=True)
    rospy.Subscriber(USER_ACTIONS, UserAction, action_callback)

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        r.sleep()
        

 
