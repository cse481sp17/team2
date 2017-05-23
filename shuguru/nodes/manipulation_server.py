#!/usr/bin/env python

import os 
import pickle
import tf
import fetch_api
import numpy as np
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
from shuguru.srv import PutBox, GrabBox

AR_POSE = "/ar_pose_marker"
DATA_PATH = "../data/"

markers = []
grab_poses = []
put_poses = []

def handle_manipulate_box(req, poses):
    global markers
    gripper = fetch_api.Gripper()

    for action in poses:
        # Compute the pose of the wrist in base_link
        ar = fetch_api.pose2matrix(action.arPose)

        ar2wrist = fetch_api.pose2transform(action.arPose, action.wristPose, True)

        wrist = np.dot(fetch_api.pose2matrix(filter(lambda x: x.id == req.ar_id, markers)[0].pose.pose), ar2wrist)

        # Navigate the arm there
        kwargs = {
            'allowed_planning_time': 50,
            'execution_timeout': 40,
            'num_planning_attempts': 30,
            'replan': False,
        }
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.pose = fetch_api.matrix2pose(wrist)
        arm.move_to_pose(pose_stamped, **kwargs)

        # Wait a second/100
    return


def handle_grab_box(req):
    global grab_poses
    handle_manipulate_box(req, grab_pose)
    return


def handle_put_box(req):
    global put_poses
    handle_manipulate_box(req, pub_poses)
    return


def arCallback(msg):
    global markers
    markers = msg.markers


def load(fileName):
    if len(fileName) == 0:
        print("Invalid Name: Empty string")
        return
    try:
        actions = pickle.load(open(fileName, 'rb'))
    except:
        actions = []
    return actions


def main():
    rospy.init_node('manipulation_server')
    ar_sub = rospy.Subscriber(AR_POSE, AlvarMarkers, arCallback)

    global grab_poses
    global put_poses
    grab_poses = load(DATA_PATH + "grab_box.dat")
    put_poses = load(DATA_PATH + "put_box.dat")

    rospy.Service('put_box', PutBox, handle_put_box)
    print("Ready to put boxes.")
    rospy.Service('grab_box', GrabBox, handle_grab_box)
    print("Ready to grab boxes.")

    rospy.spin()


if __name__ == "__main__":
    main()
