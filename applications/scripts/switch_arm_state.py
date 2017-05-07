#!/usr/bin/env python

import rospy
import fetch_api
import actionlib
from robot_controllers_msgs.msg import ControllerState, QueryControllerStatesAction, QueryControllerStatesGoal

class ControllerStatesClient(object):
    def __init__(self):
        self._client = actionlib.SimpleActionClient('/query_controller_states', QueryControllerStatesAction)
        self._client.wait_for_server()

    def _switch(self, command):
        state = ControllerState()
        state.name = 'arm_controller/follow_joint_trajectory'
        state.state = command
        goal = QueryControllerStatesGoal()
        goal.updates.append(state)
        self._client.send_goal(goal)
        self._client.wait_for_result()
        result = self._client.get_result()

    def start(self):
        self._switch(ControllerState.RUNNING)

    def relax(self):
        self._switch(ControllerState.STOPPED)


if __name__ == '__main__':

    rospy.init_node('switch_arm_state')
    client = ControllerStatesClient()
    r = rospy.Rate(10)
    while(True):
        cin = raw_input("> ")
        if cin == "start":
            client.start()
        elif cin == "relax":
            client.relax()
        elif cin == "quit":
            exit(0)
        else:
            print "options: start, relax, quit"
        r.sleep()

