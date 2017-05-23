#!/usr/bin/env python

import os 
import pickle
import tf
import fetch_api
import numpy as np
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
from sensor_msgs.msg import PointCloud2
from shuguru.srv import PutBox, GrabBox

AR_POSE = "/ar_pose_marker"
POINT_CLOUD = "/head_camera/depth_registered/points"
CURRENT_POINT_CLOUD = "/current_point_cloud"
DATA_PATH = "../data/"

markers = []
grab_poses = []
put_poses = []
pc_pub = None


def handle_grab_box(req):
    global markers
    global grab_poses
    global pc_pub

    # Update the Point Cloud
    pc = rospy.wait_for_message(POINT_CLOUD, PointCloud2)
    pc_pub.publish(pc)

    # Wait until markers are updated
    for i in range(3):
        print("Waiting for the markers... %d/3".format(i))
        if any(markers):
            break
        if i == 2:
            return 1
        rospy.sleep(1.0)

    # Navigate the gripper
    arm = fetch_api.Arm()
    for action in grab_poses:
        # Compute the pose of the wrist in base_link
        ar = fetch_api.pose2matrix(action.arPose)

        ar2wrist = fetch_api.pose2transform(action.arPose, action.wristPose, True)
        target_marker = filter(lambda x: x.id == req.ar_id, markers)
        if not any(target_marker):
            markers = []
            return 1

        wrist = np.dot(fetch_api.pose2matrix(target_marker[0].pose.pose), ar2wrist)

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

    # Empty markers for the next call
    markers = []
    return 0


def handle_put_box(req):
    global put_poses

    kwargs = {
        'allowed_planning_time': 50,
        'execution_timeout': 40,
        'num_planning_attempts': 30,
        'replan': False,
    }

    DISCO_POSES = [[1.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0],
                   [0.8, 0.75, 0.0, -2.0, 0.0, 2.0, 0.0],
                   [-0.8, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0],
                   [-1.5, 1.1, -3.0, -0.5, -3.0, -1.0, -3.0],
                   [-0.8, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0],
                   [0.8, 0.75, 0.0, -2.0, 0.0, 2.0, 0.0],
                   [1.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0]]

    torso = fetch_api.Torso()
    torso.set_height(fetch_api.Torso.MAX_HEIGHT)
    
    arm = fetch_api.Arm()
    for pose in DISCO_POSES:
        arm.move_to_joints(fetch_api.ArmJoints.from_list(pose))
    return 0


def arCallback(msg):
    global markers
    markers = msg.markers


def load(fileName):
    if len(fileName) == 0:
        return
    try:
        actions = pickle.load(open(fileName, 'rb'))
    except:
        actions = []
    return actions


def main():
    global grab_poses
    global put_poses
    global pc_pub

    rospy.init_node('manipulation_server')

    pc_pub = rospy.Publisher(CURRENT_POINT_CLOUD, PointCloud2, queue_size=10)
    ar_sub = rospy.Subscriber(AR_POSE, AlvarMarkers, arCallback)

    grab_poses = load(DATA_PATH + "grab_box.dat")
    put_poses = load(DATA_PATH + "put_box.dat")

    rospy.Service('grab_box', GrabBox, handle_grab_box)
    print("Ready to grab boxes.")
    rospy.Service('put_box', PutBox, handle_put_box)
    print("Ready to put boxes.")

    rospy.spin()


if __name__ == "__main__":
    main()

