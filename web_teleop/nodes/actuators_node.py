#!/usr/bin/env python

import fetch_api
import rospy
from web_teleop.srv import SetTorso, SetTorsoResponse, SetGripper, SetGripperResponse, SetArm, SetArmResponse, SetHead, SetHeadResponse


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class ActuatorServer(object):
    def __init__(self):
        self._torso = fetch_api.Torso()
        self._arm = fetch_api.Arm()
        self._gripper = fetch_api.Gripper()
        self._head = fetch_api.Head()

    def handle_set_torso(self, request):
        # TODO: move the torso to the requested height
        self._torso.set_height(request.height)
        return SetTorsoResponse()

    def handle_set_arm(self, request):
        self._arm.move_to_joints(fetch_api.ArmJoints.from_list(request.arm_joints))
        return SetArmResponse()

    def handle_set_gripper(self, request):
        if (request.position > 0.05):
            self._gripper.open()
        else:
            self._gripper.close(request.max_effort)
        return SetGripperResponse()
"""
    def handle_set_head(self, request):
        # TODO
        return SetHeadResponse()
"""

def main():
    rospy.init_node('web_teleop_actuators')
    wait_for_time()
    server = ActuatorServer()
    torso_service = rospy.Service('web_teleop/set_torso', SetTorso,
                                  server.handle_set_torso)
    gripper_service = rospy.Service('web_teleop/set_gripper', SetGripper,
                                    server.handle_set_gripper)
    arm_service = rospy.Service('web_teleop/set_arm', SetArm,
                                    server.handle_set_arm)
    rospy.spin()


if __name__ == '__main__':
    main()
