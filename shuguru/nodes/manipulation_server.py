#!/usr/bin/env python

import os 
import json
import tf
import fetch_api
import numpy as np
import rospy
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
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
MOTION_PLAN_GOAL = "/motion_plan_goal"
CURRENT_POINT_CLOUD = "/current_point_cloud"
DATA_PATH = "/home/team2/catkin_ws/src/cse481c/shuguru/data"
INITIAL_POSE = [1.32,1.4, -0.2, 1.72, 0,1.086, 0]
PREPARE_POSE = [-0.0482, 1.51, 3.091, 2.056, 3.04, 0.57, 0.0]
DROP_BOX_POSE = [-0.115, 1.432, 2.97, 1.91, 3.06, 1.10, 0.0]
SHELF_HEAD_POSE = [0.0,0.2985]
MOVING_HEAD_POSE = [0.0,0.0]

markers = []
grab_poses = []
put_poses = []
pc_pub = None
robot_pose = None

def distance(before, after):
    return ((before.x - after.x)**2
                + (before.y - after.y)**2)**0.5

def handle_grab_box(req):
    """ 
    Given a req, grabs the shoe box at the shoe shelf
    """
    global markers
    global grab_poses
    global pc_pub
    global viz_pub
    global robot_pos
    
    arm = fetch_api.Arm()
    gripper = fetch_api.Gripper()
    torso = fetch_api.Torso()
    head = fetch_api.Head()
    base = fetch_api.Base()

    # Prepare for grabbing the box
    print("Preparing to Grab Box, setting height + arm")

    # Move the arm, gripper, toros and head to initial position
    gripper.open()
    beginHeight = torso.MIN_HEIGHT

    # Move torso higher if on top shelf:
    if req.ar_id == 6 or req.ar_id == 7:
        beginHeight = 0.2
    torso.set_height(beginHeight)

    head.pan_tilt(*SHELF_HEAD_POSE)
    arm.move_to_joints(fetch_api.ArmJoints.from_list(PREPARE_POSE))
    rospy.sleep(1.0)

    # Update the Point Cloud
    pc = rospy.wait_for_message(POINT_CLOUD, PointCloud2)
    pc_pub.publish(pc)

    # Wait 3 seconds until markers are updated
    for i in range(3):
        print("Waiting for the markers... {}/3".format(i))
        if any(markers):
            break
        if i == 2:
            print("Didn't find any marker")
            arm.move_to_joints(fetch_api.ArmJoints.from_list(INITIAL_POSE))
            rospy.sleep(1.0)
            return 1
        rospy.sleep(1.0)

    # Set the target
    target_marker = filter(lambda x: x.id == req.ar_id, markers)

    print [marker.id for marker in markers]
    print("Target: {}".format(req.ar_id))

    if not any(target_marker):
        print("Didn't find the target marker")
        markers = []
        arm.move_to_joints(fetch_api.ArmJoints.from_list(INITIAL_POSE))
        rospy.sleep(1.0)
        return 1
    # Navigate the gripper
    print("Navigating arm to the target box")
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

            mp_marker = Marker()
            mp_marker.header.frame_id = "base_link"
            mp_marker.header.stamp = rospy.Time()
            mp_marker.id = 0;
            mp_marker.type = Marker.ARROW
            mp_marker.action = Marker.ADD
            mp_marker.ns = 'motion_plan_goal'
            mp_marker.pose.position.x = pose_stamped.pose.position.x
            mp_marker.pose.position.y = pose_stamped.pose.position.y
            mp_marker.pose.position.z = pose_stamped.pose.position.z
            mp_marker.pose.orientation.x = pose_stamped.pose.orientation.x
            mp_marker.pose.orientation.y = pose_stamped.pose.orientation.y
            mp_marker.pose.orientation.z = pose_stamped.pose.orientation.z
            mp_marker.pose.orientation.w = pose_stamped.pose.orientation.w
            mp_marker.scale.x = 0.2
            mp_marker.scale.y = 0.05
            mp_marker.scale.z = 0.05;
            mp_marker.color.a = 1.0;
            mp_marker.color.r = 1.0;
            mp_marker.color.g = 0.0;
            mp_marker.color.b = 0.0;
            viz_pub.publish(mp_marker);

            error = arm.move_to_pose(pose_stamped, **kwargs)
            if error is None:
                rospy.loginfo('Moved to the target marker')
            else:
                rospy.logwarn('Failed to move to the target marker')
                arm.move_to_joints(fetch_api.ArmJoints.from_list(INITIAL_POSE))
                rospy.sleep(1.0)
                return 1

            rospy.sleep(1.0)

        elif action[0] == 'OPEN':
            gripper.open()

        elif action[0] == 'CLOSE':
            gripper.close()

    gripper.close()

    print("Grabbed the box, moving torso up")
    torso.set_height(beginHeight + 0.1)

    # Back up the base, set torso back down and head back up
    head.pan_tilt(*MOVING_HEAD_POSE)
    before_pos = robot_pose.position
    after_pos = robot_pose.position
    while distance(before_pos, after_pos) < 0.15:
        after_pos = robot_pose.position
        base.move(-0.1, 0.0)

    torso.set_height(torso.MIN_HEIGHT)

    # Set arm to initial position
    arm.move_to_joints(fetch_api.ArmJoints.from_list(PREPARE_POSE))
    rospy.sleep(1.0)

    # Empty markers for the next call
    markers = []
    return 0


def handle_put_box(req):
    """
    Robot arrived at station, drop the box.
    """
    global put_poses

    torso = fetch_api.Torso()
    arm = fetch_api.Arm()
    torso.set_height(torso.MIN_HEIGHT)
    gripper = fetch_api.Gripper()
    base = fetch_api.Base()

    # Drop the box 
    print("Dropping the box")
    arm.move_to_joints(fetch_api.ArmJoints.from_list(DROP_BOX_POSE))
    rospy.sleep(1.0)
    gripper.open()
    rospy.sleep(0.5)

    # Move to intial pose
    arm.move_to_joints(fetch_api.ArmJoints.from_list(INITIAL_POSE))

    # Move back 
    print("Dropped box, backing up")
    before_pos = robot_pose.position
    after_pos = robot_pose.position
    while distance(before_pos, after_pos) < 0.15:
        after_pos = robot_pose.position
        base.move(-0.1, 0.0)
        
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
        if action["action"] == 0:
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

        elif action["action"] == 1:
            grab_poses.append(['OPEN'])

        elif action["action"] == 2:
            grab_poses.append(['CLOSE'])

        else:
            print("Invalid action file")
            exit(0)


def main():
    global pc_pub
    global viz_pub

    rospy.init_node('manipulation_server')

    pc_pub = rospy.Publisher(CURRENT_POINT_CLOUD, PointCloud2, queue_size=10)
    viz_pub = rospy.Publisher(MOTION_PLAN_GOAL, Marker, queue_size=10)
    ar_sub = rospy.Subscriber(AR_POSE, AlvarMarkers, arCallback)
    pose_sub = rospy.Subscriber(AMCL_POSE, PoseWithCovarianceStamped, poseCallback)

    load(DATA_PATH + "/grab_box.json")

    rospy.Service('grab_box', GrabBox, handle_grab_box)
    print("Ready to grab boxes.")
    rospy.Service('put_box', PutBox, handle_put_box)
    print("Ready to put boxes.")

    rospy.spin()


if __name__ == "__main__":
    main()

