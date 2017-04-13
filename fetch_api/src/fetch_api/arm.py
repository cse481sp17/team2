import rospy
import actionlib
import control_msgs.msg
import trajectory_msgs.msg
from .arm_joints import ArmJoints

ACTION_NAME = 'arm_controller/follow_joint_trajectory'
TIME_FROM_START = 5

class Arm(object):
    """Arm controls the robot's arm.

    Joint space control:
        joints = ArmJoints()
        # Fill out joint states
        arm = fetch_api.Arm()
        arm.move_to_joints(joints)
    """

    def __init__(self):
        # TODO: Create actionlib client
	self.client = actionlib.SimpleActionClient(ACTION_NAME, control_msgs.msg.FollowJointTrajectoryAction)
        # TODO: Wait for server
	self.client.wait_for_server()

    def move_to_joints(self, arm_joints):
        """Moves the robot's arm to the given joints.

        Args:
            arm_joints: An ArmJoints object that specifies the joint values for
                the arm.
        """
        # TODO: Create a trajectory point
	points = trajectory_msgs.msg.JointTrajectoryPoint()
        # TODO: Set position of trajectory point
	points.positions = arm_joints.values()
        # TODO: Set time of trajectory point
	points.time_from_start = rospy.Duration(TIME_FROM_START)

        # TODO: Create goal
	goal = control_msgs.msg.FollowJointTrajectoryGoal()
        # TODO: Add joint name to list
	goal.trajectory.joint_names = ArmJoints.names()
        # TODO: Add the trajectory point created above to trajectory
	goal.trajectory.points.append(points)

        # TODO: Send goal
	self.client.send_goal(goal)
        # TODO: Wait for result
	self.client.wait_for_result()
