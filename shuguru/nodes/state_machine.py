#!/usr/bin/env python

import rospy
import pickle
from shuguru.srv import GetCommand, PutCommand
from smach import State, StateMachine
from smach_ros import SimpleActionState, IntrospectionServer
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import PointCloud2
from ar_track_alvar_msgs.msg import AlvarMarkers

MOVE_BASE = '/move_base'
GET_COMMAND = '/next_command'
POSES = '/home/team2/catkin_ws/src/cse481c/shuguru/data/dests.dat'
GET_CLOUD_TOPIC = 'head_camera/depth_registered/points'
PUBLISH_CLOUD_TOPIC = 'current_point_cloud'
AR_MARKER_TOPIC = 'ar_pose_marker'

markers = []

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
    def __init__(self, point_cloud_publisher):
        State.__init__(self,
                       outcomes=['succeeded', 'preempted', 'aborted'],
                       input_keys=['shoe_id'],
                       output_keys=[])
        self.point_cloud_publisher = point_cloud_publisher

    def execute(self, userdata):
        '''TODO
            1. Save latest point cloud
            2. Detect fiducials in it
            3. Grab box
        '''

        # TODO: idk how to avoid these globals, but maybe we
        # can get rid of them somehow?
        global markers
        point_cloud = rospy.wait_for_message(GET_CLOUD_TOPIC, PointCloud2)

        point_cloud_publisher.publish(point_cloud)

        # wait for markers to be published
        while not markers
            rospy.sleep(0.5)

        '''TODO
            Arm stuff
        '''

        '''
            Delete markers after finishing movement,
            so next go around we make sure to get a
            new set
        '''

        markers = []

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
    goal.target_pose.pose = userdata.goal_poses[str(userdata.goal_id)]
    return goal


def load_poses(load_path):
    poses = pickle.load(open(load_path, 'rb'))
    print(poses)
    return poses

def marker_callback(self, msg):
    global markers
    markers = msg.markers

def main():
    rospy.init_node('state_machine')
    sm = StateMachine(['succeeded', 'preempted', 'aborted'])

    point_cloud_publisher = rospy.Publisher(PUBLISH_CLOUD_TOPIC, PointCloud2, queue_size=10)

    markers_subscriber = rospy.Subscriber(AR_MARKER_TOPIC, AlvarMarkers, marker_callback)

    # load pre-defined goal poses from pickle file
    sm.userdata.goal_poses = load_poses(POSES)
    print(sm.userdata.goal_poses)

    # goal_id:  0    , 1     , 2     , ..., N
    #           shelf, seat#1, seat#2, ..., seat#N
    sm.userdata.goal_id = 0
    sm.userdata.shoe_id = 0

    # Register states to the state machine
    # TODO: Are state objects created and destroyed multiple times?
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
                         GrabBoxState(point_cloud_publisher),
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
