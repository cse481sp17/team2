#!/usr/bin/env python

import rospy
import pickle
from shuguru.srv import GetCommand, PutCommand
from smach import State, StateMachine
from smach_ros import SimpleActionState, IntrospectionServer
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


MOVE_BASE = '/move_base'
GET_COMMAND = '/next_command'
POSES = '/home/team2/catkin_ws/src/cse481c/shuguru/data/poses.p'


class GetCommandState(State):
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
            '''TODO
                request command via service
                command = brabra
            '''
            command = get_command()
            rospy.sleep(1.0)
        userdata.goal_id = str(command.goal_id)
        userdata.shoe_id = str(command.shoe_id)
        return 'succeeded'


class GrabBoxState(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'preempted', 'aborted'],
                       input_keys=['shoe_id'],
                       output_keys=[])

    def execute(self, userdata):
        '''TODO
            Do something to grab shoe box
        '''
        rospy.loginfo('Executing state GRAB_BOX')
        return 'succeeded'


class PutBoxState(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'preempted', 'aborted'],
                       input_keys=['goal_id'],
                       output_keys=['goal_id'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PUT_BOX')
        '''TODO
            Do something to put shoe box
        '''
        userdata.goal_id = 0  # Reset the goal to the shelf
        return 'succeeded'


def goal_callback(userdata, goal):
    goal = MoveBaseGoal()
    goal.target_pose.header.stamp = rospy.Time()
    goal.target_pose.header.frame_id = "map"
    print(userdata.goal_poses)
    print(userdata.goal_id)
    goal.target_pose.pose = userdata.goal_poses[str(userdata.goal_id)]
    return goal


def load_poses(load_path):

    print(load_path)
    poses = pickle.load(open(load_path, 'rb'))
    return poses


def main():
    rospy.init_node('state_machine')
    sm = StateMachine(['succeeded', 'preempted', 'aborted'])

    # load pre-defined goal poses from pickle file
    sm.userdata.goal_poses = load_poses(POSES)
    print(sm.userdata.goal_poses)

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
