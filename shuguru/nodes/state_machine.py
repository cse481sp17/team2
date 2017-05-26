#!/usr/bin/env python

import rospy
import pickle
from smach import State, StateMachine
from smach_ros import SimpleActionState, IntrospectionServer
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import PointCloud2
from ar_track_alvar_msgs.msg import AlvarMarkers
from shuguru.srv import GetCommand, PutCommand, GrabBox, PutBox

"""
Handles the logic + transition from different states of the 
fetch robot operation. 
"""

MOVE_BASE = '/move_base'
# Defines the shelf and different shoe box dropoff locations
POSES = '/home/team2/catkin_ws/src/cse481c/shuguru/data/dests.dat' 

markers = []

class GetCommandState(State):
    """
    State when user orders a shoe box and that order is forwarded
    to the robot
    """
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'preempted', 'aborted'],
                       input_keys=['goal_id', 'shoe_id'],
                       output_keys=['goal_id', 'shoe_id'])

    def execute(self, userdata):
        rospy.loginfo('Executing state GET_COMMAND')

        rospy.wait_for_service('get_command')
        get_command = rospy.ServiceProxy('get_command', GetCommand)
        
        command = None
        while not command:
            command = get_command()
            rospy.sleep(1.0)
        userdata.goal_id = str(command.goal_id)
        userdata.shoe_id = str(command.shoe_id)
        return 'succeeded'


class GrabBoxState(State):
    """
    State when the robot arrives at the shelf and needs to grab a shoe
    box
    """
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'preempted', 'aborted'],
                       input_keys=['shoe_id'],
                       output_keys=[])

    def execute(self, userdata):
        rospy.loginfo('Executing state GRAB_BOX')

        print("Waiting for grab_box service")
        rospy.wait_for_service('grab_box')
        print("Created service proxy")
        grab_box = rospy.ServiceProxy('grab_box', GrabBox)

        # result = grab_box(int(userdata.shoe_id))
        # TODO: Why grab box with ID of 7? Rio
        # TODO: Return aborted if unsuccessful 
        result = grab_box(7)
        if result == 0:
            return 'succeeded'
        return 'succeeded'


class PutBoxState(State):
    """
    State when robot has arrived at a customer designated area
    and needs to drop the shoe box
    """
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'preempted', 'aborted'],
                       input_keys=['goal_id'],
                       output_keys=['goal_id'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PUT_BOX')

        rospy.wait_for_service('put_box')
        put_box = rospy.ServiceProxy('put_box', PutBox)

        userdata.goal_id = 0  # Reset the goal to the shelf
        result = put_box()

        # TODO: Return aborted if unsuccessful 
        if result == 0:
            return 'succeeded'
        return 'succeeded'


def goal_callback(userdata, goal):
    """
    Sets the goal of the robot to go to
    """
    goal = MoveBaseGoal()
    goal.target_pose.header.stamp = rospy.Time()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose = userdata.goal_poses[str(userdata.goal_id)]
    return goal


def load_poses(load_path):
    with open(load_path, 'rb') as f:
        poses = pickle.load(f)
    return poses

def marker_callback(self, msg):
    global markers
    markers = msg.markers

def main():
    rospy.init_node('state_machine')
    sm = StateMachine(['succeeded', 'preempted', 'aborted'])

    # load pre-defined goal poses from pickle file
    sm.userdata.goal_poses = load_poses(POSES)
    print(sm.userdata.goal_poses)


    # TODO: Change the goal_id and shoe_id later
    # goal_id:  0    , 1     , 2     , ..., N
    #           shelf, seat#1, seat#2, ..., seat#N
    sm.userdata.goal_id = 0
    sm.userdata.shoe_id = 0

    # Register states to the state machine
    with sm:
        StateMachine.add('GO_SHELF',
                         SimpleActionState(
                             MOVE_BASE,
                             MoveBaseAction,
                             goal_cb=goal_callback,
                             input_keys=['goal_poses', 'goal_id'],
                             output_keys=[]),
                         transitions={'succeeded': 'GET_COMMAND',
                                      'preempted': 'preempted',
                                      'aborted': 'aborted'},
                         remapping={})

        StateMachine.add('GET_COMMAND',
                         GetCommandState(),
                         transitions={'succeeded': 'GRAB_BOX',
                                      'preempted': 'preempted',
                                      'aborted': 'aborted'},
                         remapping={})

        StateMachine.add('GRAB_BOX',
                         GrabBoxState(),
                         transitions={'succeeded': 'GO_SEAT',
                                      'preempted': 'preempted',
                                      'aborted': 'aborted'},
                         remapping={})

        StateMachine.add('GO_SEAT',
                         SimpleActionState(
                             MOVE_BASE,
                             MoveBaseAction,
                             goal_cb=goal_callback,
                             input_keys=['goal_poses', 'goal_id'],
                             output_keys=[]),
                         transitions={'succeeded': 'PUT_BOX',
                                      'preempted': 'preempted',
                                      'aborted': 'aborted'},
                         remapping={})

        StateMachine.add('PUT_BOX',
                         PutBoxState(),
                         transitions={'succeeded': 'GO_SHELF',
                                      'preempted': 'preempted',
                                      'aborted': 'aborted'},
                         remapping={})

        # Create an instrospection server
        sis = IntrospectionServer('smach_viz', sm, '/SM_ROOT')
        sis.start()

        # Start the state machine, and kill everything when completed
        sm.execute()
        rospy.signal_shutdown('All done.')


if __name__ == '__main__':
    main()
