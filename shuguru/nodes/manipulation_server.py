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
from moveit_python import PlanningSceneInterface

"""
Handles the robot action of grabbing a specifc AR tagged shoebox
and the action that puts down the shoe box.
"""

AR_POSE = "/ar_pose_marker"
AMCL_POSE = "/amcl_pose"
POINT_CLOUD = "/head_camera/depth_registered/points"
MOTION_PLAN_GOAL = "/motion_plan_goal"
DATA_PATH = "/home/team2/catkin_ws/src/cse481c/shuguru/data"

# Move Joints Poses
INITIAL_POSE = [1.32,1.4, -0.2, 1.72, 0,1.086, 0]
PREPARE_POSE = [-0.0482, 1.51, 3.091, 2.056, 3.04, 0.57, 0.0]
#PREPARE_POSE = [-0.80, 1.51, -2.84, 2.24, -1.93, 1.19, -0.75] # Sideways prepare
CARRY_POSE = [-1.57, 0.68, 0.21, 1.08, -2.92, -1.33, 0.05]
DROP_BOX_POSE = [-0.115, 1.432, 2.97, 1.91, 3.06, 1.10, 0.0]
SHELF_HEAD_POSE = [0.0,0.2985]
MOVING_HEAD_POSE = [0.0,0.0]

# Move to pose Pose
PREPARE = [0.372, -0.32, 0.706, 0.011, -0.014, -0.023, 1]
CARRY = [0.25, -0.255, 0.712, 0.003, -0.006, 0.674,0.738]
DROP = [0.277, -0.361, 0.591, -0.090, 0.304, 0.215, 0.924]

markers = []
grab_poses = []
put_poses = []
robot_pose = None

# Grabber Planning Scene
planning= PlanningSceneInterface('base_link')

def distance(before, after):
    return ((before.x - after.x)**2
                + (before.y - after.y)**2)**0.5

def move_dist(dist, speed):  
    """
    Move a dist direction based on speed
    """
    base = fetch_api.Base()
    before_pos = robot_pose.position
    after_pos = robot_pose.position
    while distance(before_pos, after_pos) < dist:
        after_pos = robot_pose.position
        base.move(speed, 0.0)

def move_pose(arm, xyz):
    kwargs = {
        'allowed_planning_time': 50,
        'execution_timeout': 40,
        'num_planning_attempts': 30,
        'replan': False,
    }
    pose_stampeda = PoseStamped()
    pose_stampeda.header.frame_id = "base_link"
    pose_stampeda.pose.position.x = xyz[0]
    pose_stampeda.pose.position.y = xyz[1]
    pose_stampeda.pose.position.z = xyz[2]
    pose_stampeda.pose.orientation.x = xyz[3]
    pose_stampeda.pose.orientation.y = xyz[4]
    pose_stampeda.pose.orientation.z = xyz[5]
    pose_stampeda.pose.orientation.w =  xyz[6]
    arm.move_to_pose(pose_stampeda, **kwargs)

def handle_grab_box(req):
    """ 
    Given a req, grabs the shoe box at the shoe shelf
    """
    global markers
    global grab_poses
    global viz_pub
    global robot_pose
    global planning
    
    arm = fetch_api.Arm()
    gripper = fetch_api.Gripper()
    torso = fetch_api.Torso()
    head = fetch_api.Head()
    base = fetch_api.Base()

    # Prepare for grabbing the box
    print("Preparing to Grab Box, setting height + arm")

    # Move the arm, gripper, toros and head to initial position
    gripper.open()
    beginHeight = 0.1

    # Navigate the arm to prepare
    move_pose(arm, PREPARE)

    # Move torso higher if on top shelf:
    if req.ar_id == 6 or req.ar_id == 4:
        beginHeight = 0.2
    torso.set_height(beginHeight)

    head.pan_tilt(*SHELF_HEAD_POSE)
    rospy.sleep(3.0)

    # Wait 5 seconds until markers are updated
    target_marker = None
    for i in range(5):
        print("Waiting for the markers... {}/3".format(i))
        if any(markers):
            print([marker.id for marker in markers])
            target_marker = filter(lambda x: x.id == req.ar_id, markers)
            if any(target_marker):
                break
        if i == 4:
            print("Didn't find any marker")
            move_dist(0.1, -0.1)
            torso.set_height(torso.MAX_HEIGHT)
            move_pose(arm, CARRY)
            rospy.sleep(1.0)
            return 1
        rospy.sleep(1.0)

    print [marker.id for marker in markers]
    print("Target: {}".format(req.ar_id))

    if not any(target_marker):
        print("Didn't find the target marker")
        markers = []
        move_dist(0.1, -0.1)
        torso.set_height(torso.MAX_HEIGHT)
        move_pose(arm, CARRY)
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
                move_dist(0.1, -0.1)
                torso.set_height(torso.MAX_HEIGHT)
                move_pose(arm, CARRY)
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
    """
    while distance(before_pos, after_pos) < 0.19:
        after_pos = robot_pose.position
        base.move(-0.1, 0.0)
    """
    move_dist(0.19, -0.1)

    torso.set_height(torso.MAX_HEIGHT)

    # Set arm to initial position
    move_pose(arm, CARRY)

    # Empty markers for the next call
    return 0


def handle_put_box(req):
    """
    Robot arrived at station, drop the box.
    """
    global put_poses

    torso = fetch_api.Torso()
    arm = fetch_api.Arm()
    gripper = fetch_api.Gripper()
    base = fetch_api.Base()

    # Drop the box 
    print("Dropping the box")
    # arm.move_to_joints(fetch_api.ArmJoints.from_list(DROP_BOX_POSE))
    move_pose(arm, DROP)
    rospy.sleep(1.0)
    gripper.open()
    rospy.sleep(0.5)
    #torso.set_height(torso.MIN_HEIGHT)
    
    # Move to carry pose
    move_pose(arm, CARRY)
    #arm.move_to_joints(fetch_api.ArmJoints.from_list(INITIAL_POSE))

    # Move back 
    print("Dropped box, backing up")
    before_pos = robot_pose.position
    after_pos = robot_pose.position
    while distance(before_pos, after_pos) < 0.18:
        after_pos = robot_pose.position
        base.move(-0.1, 0.0)
        
    return 0

def arCallback(msg):
    global markers
    if len(msg.markers) > len(markers):
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
def attach_grabber():
    global planning
    frame_attached_to = 'gripper_link'
    frames_okay_to_collide_with = [ 'gripper_link',
            'l_gripper_finger_link',
            'r_gripper_finger_link']
    planning.attachBox('tray', 0.34,0.3, 0.14, 0.22, 0,0.03,
                        frame_attached_to,
                        frames_okay_to_collide_with)
    planning.setColor('tray',1,0,1)
    planning.sendColors()

def remove_grabber():
    global planning
    planning.removeAttachedObject('tray')

def main():
    global viz_pub
    global planning

    rospy.init_node('manipulation_server')
    remove_grabber()
    attach_grabber()

    torso = fetch_api.Torso()
    arm = fetch_api.Arm()
    torso.set_height(torso.MAX_HEIGHT)
    move_pose(arm, CARRY)

    viz_pub = rospy.Publisher(MOTION_PLAN_GOAL, Marker, queue_size=10)
    ar_sub = rospy.Subscriber(AR_POSE, AlvarMarkers, arCallback)
    pose_sub = rospy.Subscriber(AMCL_POSE, PoseWithCovarianceStamped, poseCallback)

    load(DATA_PATH + "/grab_box.json")


    # Attach Grabber to planning Scene

    rospy.Service('grab_box', GrabBox, handle_grab_box)
    print("Ready to grab boxes.")
    rospy.Service('put_box', PutBox, handle_put_box)
    print("Ready to put boxes.")

    rospy.spin()


if __name__ == "__main__":
    main()

