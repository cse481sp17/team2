#!/usr/bin/env python

import actionlib
import trajectory_msgs.msg
import control_msgs.msg
import rospy

ACTION_NAME = "torso_controller/follow_joint_trajectory"
JOINT_NAME = "torso_lift_joint"
TIME_FROM_START = 5  # How many seconds it should take to set the torso height.


class Torso(object):
    """Torso controls the robot's torso height.
    """
    MIN_HEIGHT = 0.0
    MAX_HEIGHT = 0.4

    def __init__(self):
        # TODO: Create actionlib client
        self.client = actionlib.SimpleActionClient(ACTION_NAME, control_msgs.msg.FollowJointTrajectoryAction)
        # TODO: Wait for server
        self.client.wait_for_server()

    def set_height(self, height):
        """Sets the torso height.

        This will always take ~5 seconds to execute.

        Args:
            height: The height, in meters, to set the torso to. Values range
                from Torso.MIN_HEIGHT (0.0) to Torso.MAX_HEIGHT(0.4).
        """
        # TODO: Check that the height is between MIN_HEIGHT and MAX_HEIGHT.
        
        if height < Torso.MIN_HEIGHT:
            height = Torso.MIN_HEIGHT
        if height > Torso.MAX_HEIGHT:
            height = Torso.MAX_HEIGHT

        # TODO: Create a trajectory point
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        
        # TODO: Set position of trajectory point
        # TODO: Set time of trajectory point        
        point.positions.append(height)
        point.time_from_start = rospy.Duration(TIME_FROM_START)

        # TODO: Create goal
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        # TODO: Add joint name to list
        goal.trajectory.joint_names.append(JOINT_NAME)

        # TODO: Add the trajectory point created above to trajectory
        goal.trajectory.points.append(point)

        # TODO: Send goal
        self.client.send_goal(goal)

        # TODO: Wait for result
        self.client.wait_for_result()
