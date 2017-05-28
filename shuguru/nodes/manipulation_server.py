#!/usr/bin/env python

import os 
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
    global robot_pos
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

    print("Reset the torso height")
    torso = fetch_api.Torso()
    torso.set_height(0)

    INITIAL_POSE = [-0.05, 1.57, 3.09, 2.06, 3.04, 0.57, -3.01]
    arm = fetch_api.Arm()
    arm.move_to_joints(fetch_api.ArmJoints.from_list(INITIAL_POSE))
    
    # Navigate the gripper
    print("Navigating arm to the target box")
    arm = fetch_api.Arm()
    gripper = fetch_api.Gripper()
    for action in grab_poses:
        if action[0] == 'MOVE':
            arPose = action[1]
            wristPose = action[2]

            # Compute the pose of the wrist in base_link
            ar = fetch_api.pose2matrix(arPose)
            ar2wrist = fetch_api.pose2transform(arPose, wristPose, True)
            wrist = np.dot(fetch_api.pose2matrix(target_marker[0].pose.pose), ar2wrist)
            print(target_marker[0].pose.header.frame_id)

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

            rospy.sleep(1.0)

        elif action[1] == 'OPEN':
            gripper.open()

        elif action[2] == 'CLOSE':
            gripper.close()

    base = fetch_api.Base()

    def distance(before, after):
        return ((before.x - after.x)**2
                + (before.y - after.y)**2)**0.5

    before_pos = robot_pose.position
    after_pos = robot_pose.position
    while distance(before_pos, after_pos) < 0.2:
        after_pos = robot_pose.position
        base.move(0.1, 0.0)

    print("Move the torso up")
    torso = fetch_api.Torso()
    torso.set_height(0.1)

    # Back up the base
    before_pos = robot_pose.position
    after_pos = robot_pose.position
    while distance(before_pos, after_pos) < 0.2:
        after_pos = robot_pose.position
        base.move(-0.1, 0.0)

    # Empty markers for the next call
    markers = []
    return 0


def handle_put_box(req):
    global put_poses

    torso = fetch_api.Torso()
    torso.set_height(fetch_api.Torso.MAX_HEIGHT)
    
    DROP_BOX_POSE = [-0.11, 1.43, 2.97, 1.91, 3.06, 1.10, -3.01]
    arm = fetch_api.Arm()
    arm.move_to_joints(fetch_api.ArmJoints.from_list(INITIAL_POSE))

    return 0


def arCallback(msg):
    global markers
    markers = msg.markers


def poseCallback(msg):
    global robot_pose
    robot_pose = msg.pose.pose


def load(fileName):
    global grab_poses

    if len(fileName) == 0:
        return("Invalid file name")

    try:
        with open(fileName, 'r') as f:
            actions_json = json.load(f)
        print("Succeeded loading json.")
    except Exception as e:
        print("Failed loading json.", e)
        actions_json = []

    # self.actions = []
    for action in actions_json:
        if action.action == 0:
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

            grab_poses.append(['MOVE', arPose, wristPose])

        elif action.action == 1:
            grab_poses.append(['OPEN'])

        elif action.action == 2:
            grab_poses.append(['CLOSE'])

        else:
            print("Invalid action file")
            exit(0)

        # self.actions.append(ActionType(ActionSaver.MOVE, action['arId'],
        #                     arPose=arPose, wristPose=wristPose))



def main():
    global pc_pub

    rospy.init_node('manipulation_server')

    pc_pub = rospy.Publisher(CURRENT_POINT_CLOUD, PointCloud2, queue_size=10)
    ar_sub = rospy.Subscriber(AR_POSE, AlvarMarkers, arCallback)
    pose_sub = rospy.Subscriber(AMCL_POSE, PoseWithCovarianceStamped, poseCallback)

    load(DATA_PATH + "/test.p")

    rospy.Service('grab_box', GrabBox, handle_grab_box)
    print("Ready to grab boxes.")
    rospy.Service('put_box', PutBox, handle_put_box)
    print("Ready to put boxes.")

    rospy.spin()


if __name__ == "__main__":
    main()

