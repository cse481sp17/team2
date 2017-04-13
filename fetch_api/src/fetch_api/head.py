#!/usr/bin/env python

# TODO: import ?????????
# TODO: import ???????_msgs.msg
# TODO: import ??????????_msgs.msg
import actionlib
import control_msgs.msg
import trajectory_msgs.msg

import math
import rospy

LOOK_AT_ACTION_NAME = 'head_controller/follow_joint_trajectory'  # TODO: Get the name of the look-at action
PAN_TILT_ACTION_NAME = 'head_controller/point_head'  # TODO: Get the name of the pan/tilt action
PAN_JOINT = 'head_pan_joint'  # TODO: Get the name of the head pan joint
TILT_JOINT = 'head_tilt_joint'  # TODO: Get the name of the head tilt joint
PAN_TILT_TIME = 2.5  # How many seconds it should take to move the head.


class Head(object):
    """Head controls the Fetch's head.

    It provides two interfaces:
        head.look_at(frame_id, x, y, z)
        head.pan_tilt(pan, tilt) # In radians

    For example:
        head = fetch_api.Head()
        head.look_at('base_link', 1, 0, 0.3)
        head.pan_tilt(0, math.pi/4)
    """
    MIN_PAN = -math.pi/2  # TODO: Minimum pan angle, in radians.
    MAX_PAN = math.pi/2  # TODO: Maximum pan angle, in radians.
    MIN_TILT = -math.pi/2  # TODO: Minimum tilt angle, in radians.
    MAX_TILT = math.pi/4  # TODO: Maximum tilt angle, in radians.

    def __init__(self):
        # TODO: Create actionlib clients
        # TODO: Wait for both servers
        self.traj_client = actionlib.SimpleActionClient(LOOK_AT_ACTION_NAME, control_msgs.msg.FollowJointTrajectoryAction)
        self.point_client = actionlib.SimpleActionClient(PAN_TILT_ACTION_NAME, control_msgs.msg.PointHeadAction)
        self.traj_client.wait_for_server()
        self.point_client.wait_for_server()
        pass

    def look_at(self, frame_id, x, y, z):
        """Moves the head to look at a point in space.

        Args:
            frame_id: The name of the frame in which x, y, and z are specified.
            x: The x value of the point to look at.
            y: The y value of the point to look at.
            z: The z value of the point to look at.
        """
        # TODO: Create goal
        goal = control_msgs.msg.PointHeadGoal()
        """
        useless
        goal.pointing_axis.x = x
        goal.pointing_axis.y = y
        goal.pointing_axis.z = z
        goal.pointing_frame = frame_id
        """
        goal.target.header.frame_id = frame_id
        goal.target.point.x  = x
        goal.target.point.y  = y
        goal.target.point.z  = z

        goal.min_duration = rospy.Duration(1)
        # TODO: Fill out the goal (we recommend setting min_duration to 1 second)
        # TODO: Send the goal
        self.point_client.send_goal(goal)
        # TODO: Wait for result
        self.point_client.wait_for_result()

    def pan_tilt(self, pan, tilt):
        """Moves the head by setting pan/tilt angles.

              Args:
            pan: The pan angle, in radians. A positive value is clockwise.
            tilt: The tilt angle, in radians. A positive value is downwards.
        """
        # TODO: Check that the pan/tilt angles are within joint limits
        if pan <Head.MIN_PAN:
            pan = Head.MIN_PAN
        if pan > Head.MAX_PAN:
            pan = Head.MAX_PAN
        if tilt < Head.MIN_TILT:
            tilt = Head.MIN_TILT
        if tilt > Head.MAX_TILT:
            tilt = Head.MAX_TILT

        # TODO: Create a trajectory point
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        # TODO: Set positions of the two joints in the trajectory point
        # point.positions.append(pan)
        # point.positions.append(tilt)
        point.positions = [pan, tilt]

        # TODO: Set time of the trajectory point
        point.time_from_start = rospy.Duration(1)
        # TODO: Create goal
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        # TODO: Add joint names to the list
        # goal.trajectory.joint_names.append(PAN_JOINT)
        # goal.trajectory.joint_names.append(TILT_JOINT)

        goal.trajectory.joint_names = [PAN_JOINT, TILT_JOINT]
        # TODO: Add trajectory point created above to trajectory
        goal.trajectory.points.append(point)

        self.traj_client.send_goal(goal)

        # TODO: Wait for result
        self.traj_client.wait_for_result()