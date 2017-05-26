#!/usr/bin/env python

import os 
import dill
import json
import tf
import fetch_api
import numpy as np
import rospy
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
from sensor_msgs.msg import PointCloud2
from shuguru.srv import PutBox, GrabBox

"""
Handles the robot action of grabbing a specifc AR tagged shoebox
and the action that puts down the shoe box.
"""

AR_POSE = "/ar_pose_marker"
AMCL_POSE = "/amcl_pose"
POINT_CLOUD = "/head_camera/depth_registered/points"
CURRENT_POINT_CLOUD = "/current_point_cloud"
DATA_PATH = "/home/team2/catkin_ws/src/cse481c/shuguru/data"

markers = []
grab_poses = []
put_poses = []
pc_pub = None
robot_pose = None

def handle_grab_box(req):
    """ 
    Given a req, grabs the shoe box at the shoe shelf
    """
    global markers
    global grab_poses
    global pc_pub
    global robot_pose

    def distance(before, after):
        return ((before.x - after.x)**2
                + (before.y - after.y)**2)**0.5

    # Update the Point Cloud
    pc = rospy.wait_for_message(POINT_CLOUD, PointCloud2)
    pc_pub.publish(pc)

    # TODO: Change 3 to be reflect number of 
    # Wait until markers are updated
    for i in range(3):
        print("Waiting for the markers... {}/3".format(i))
        if any(markers):
            break
        if i == 2:
            print("Didn't find any marker")
            return 1
        rospy.sleep(1.0)

    # Set the target
    target_marker = filter(lambda x: x.id == req.ar_id, markers)

    print [marker.id for marker in markers]
    print("Target: {}".format(req.ar_id))

    if not any(target_marker):
        print("Didn't find the target marker")
        markers = []
        return 1

    STRAIGHT = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    arm = fetch_api.Arm()
    arm.move_to_joints(fetch_api.ArmJoints.from_list(STRAIGHT))
    rospy.sleep(0.5)

    # Navigate the gripper
    print("Navigating arm to the target box")
    print(grab_poses)
    for action in grab_poses:
        arPose = Pose()
        arPose.position.x = action['ar'][0]
        arPose.position.y = action['ar'][1]
        arPose.position.z = action['ar'][2]
        arPose.orientation.x = action['ar'][3]
        arPose.orientation.y = action['ar'][4]
        arPose.orientation.z = action['ar'][5]
        arPose.orientation.w = action['ar'][6]

        wristPose = Pose()
        wristPose.position.x = action['wrist'][0]
        wristPose.position.y = action['wrist'][1]
        wristPose.position.z = action['wrist'][2]
        wristPose.orientation.x = action['wrist'][3]
        wristPose.orientation.y = action['wrist'][4]
        wristPose.orientation.z = action['wrist'][5]
        wristPose.orientation.w = action['wrist'][6]

        # Compute the pose of the wrist in base_link
        ar = fetch_api.pose2matrix(arPose)
        ar2wrist = fetch_api.pose2transform(arPose, wristPose, True)
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
        # TODO: Where is the wait? Rio

    return 0

    # Rotate to face the shelf, then move forward
    before_pos = robot_pose.position
    after_pos = robot_pose.position

    base = fetch_api.Base()

    while distance(before_pos, after_pos) < 0.5:
        base.move(0.5, 0.0)

    # Move the torso up
    torso = fetch_api.Torso()
    torso.set_height(target_marker[0].pose.pose.z)

    # Back up the base
    while distance(before_pos, after_pos) > 0.1:
        base.move(-0.5, 0.0)

    # Empty markers for the next call
    markers = []
    return 0


def handle_put_box(req):
    global put_poses

    DISCO_POSES = [[1.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0],
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


def poseCallback(msg):
    global robot_pose
    robot_pose = msg.pose.pose


def load(fileName):
    if len(fileName) == 0:
        return

    try:
        with open(fileName, 'r') as f:
            actions = json.load(f)
        print("Succeeded loading json.")
        print (actions)
    except Exception as e:
        print("Failed loading json.", e)
        actions = []
    return actions


def main():
    global pc_pub
    global grab_poses

    rospy.init_node('manipulation_server')

    pc_pub = rospy.Publisher(CURRENT_POINT_CLOUD, PointCloud2, queue_size=10)
    ar_sub = rospy.Subscriber(AR_POSE, AlvarMarkers, arCallback)
    pose_sub = rospy.Subscriber(AMCL_POSE, PoseWithCovarianceStamped, poseCallback)

    grab_poses = load(DATA_PATH + "/poke3.dat")

    rospy.Service('grab_box', GrabBox, handle_grab_box)
    print("Ready to grab boxes.")
    rospy.Service('put_box', PutBox, handle_put_box)
    print("Ready to put boxes.")

    rospy.spin()


if __name__ == "__main__":
    main()

