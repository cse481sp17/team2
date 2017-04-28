#!/usr/bin/env python

import rospy
from interactive_markers.interactive_marker_server import *
from geometry_msgs.msg import *
from visualization_msgs.msg import *
import fetch_api

GRIPPER_MESH = 'package://fetch_description/meshes/gripper_link.dae'
L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.STL'
GRIPPER_MARKER_NAME = 'gripper'

class GripperTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._im_server = im_server
        self.gripper = gripper
        self.arm = arm

        # InteractiveMarker - top level object
        gripper_im = InteractiveMarker()
        gripper_im.header.frame_id = 'base_link'
        gripper_im.name = GRIPPER_MARKER_NAME
        gripper_im.description = GRIPPER_MARKER_NAME
        gripper_im.scale = .25
        
        # Initial IM Pose
        pose = Pose()
        pose.position.x = 1
        pose.position.y = 0
        pose.position.z = 1
        pose.orientation.w = 1
        gripper_im.pose = pose

        # Markers for gripper and fingers
        gripper_marker = Marker()
        gripper_marker.type = Marker.MESH_RESOURCE
        gripper_marker.mesh_resource = GRIPPER_MESH
        gripper_marker.pose.orientation.w = 1
        gripper_marker.pose.position.x = .166
        gripper_marker.scale.x = 1
        gripper_marker.scale.y = 1
        gripper_marker.scale.z = 1
        gripper_marker.color.r = 1.0
        gripper_marker.color.a = 1.0

        l_finger_m = Marker()
        l_finger_m.type = Marker.MESH_RESOURCE
        l_finger_m.mesh_resource = L_FINGER_MESH
        l_finger_m.pose.orientation.w = 1
        l_finger_m.pose.position.y = -.06
        l_finger_m.pose.position.x = .166
        l_finger_m.scale.x = 1
        l_finger_m.scale.y = 1
        l_finger_m.scale.z = 1
        l_finger_m.color.r = 1.0
        l_finger_m.color.a = 1.0
               
        r_finger_m = Marker()
        r_finger_m.type = Marker.MESH_RESOURCE
        r_finger_m.mesh_resource = R_FINGER_MESH
        r_finger_m.pose.orientation.w = 1
        r_finger_m.pose.position.y = .06
        r_finger_m.pose.position.x = .166
        r_finger_m.scale.x = 1
        r_finger_m.scale.y = 1
        r_finger_m.scale.z = 1
        r_finger_m.color.r = 1.0
        r_finger_m.color.a = 1.0

        # This control is the dropdown menu
        gripper_control = InteractiveMarkerControl()
        gripper_control.orientation.w = 1
        gripper_control.interaction_mode = InteractiveMarkerControl.MENU
        
        # Attach the gripper to the menu
        gripper_control.markers.append(gripper_marker)
        gripper_control.markers.append(l_finger_m)
        gripper_control.markers.append(r_finger_m)

        gripper_control.always_visible = True
        gripper_im.controls.append(gripper_control)

        # Controls

        # Axis shift controls
        x_axis_control = InteractiveMarkerControl()
        x_axis_control.orientation.w = 1
        x_axis_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

        x_axis_control.always_visible = True
        gripper_im.controls.append(x_axis_control)

        y_axis_control = InteractiveMarkerControl()
        y_axis_control.orientation.w = 1
        y_axis_control.orientation.z = 1
        y_axis_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

        y_axis_control.always_visible = True
        gripper_im.controls.append(y_axis_control)

        z_axis_control = InteractiveMarkerControl()
        z_axis_control.orientation.w = 1
        z_axis_control.orientation.y = 1
        z_axis_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

        z_axis_control.always_visible = True
        gripper_im.controls.append(z_axis_control)

        # Rotation Controls
        z_axis_rotate = InteractiveMarkerControl()
        z_axis_rotate.orientation.w = 1
        z_axis_rotate.orientation.y = 1
        z_axis_rotate.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS

        z_axis_rotate.always_visible = True
        gripper_im.controls.append(z_axis_rotate)

        x_axis_rotate = InteractiveMarkerControl()
        x_axis_rotate.orientation.w = 1
        x_axis_rotate.orientation.z = 1
        x_axis_rotate.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS

        x_axis_rotate.always_visible = True
        gripper_im.controls.append(x_axis_rotate)

        y_axis_rotate = InteractiveMarkerControl()
        y_axis_rotate.orientation.w = 1
        y_axis_rotate.orientation.x = 1
        y_axis_rotate.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS

        y_axis_rotate.always_visible = True
        gripper_im.controls.append(y_axis_rotate)

        # Menu Entries
        move_menu_entry = MenuEntry()
        move_menu_entry.id = 1
        move_menu_entry.title = "Move Here"

        close_menu_entry = MenuEntry()
        close_menu_entry.id = 2
        close_menu_entry.title = "Close"

        open_menu_entry = MenuEntry()
        open_menu_entry.id = 3
        open_menu_entry.title = "Open"

        gripper_im.menu_entries.append(move_menu_entry)
        gripper_im.menu_entries.append(close_menu_entry)
        gripper_im.menu_entries.append(open_menu_entry)

        self._im_server.insert(gripper_im, feedback_cb = self.handle_feedback)
        
        self._im_server.applyChanges()


    def start(self):
        pass

    def handle_feedback(self, feedback):
        id = feedback.menu_entry_id
        if id == 1:
            # Move the arm
            pass
        elif id == 2:
            # Close the gripper
            self.gripper.close()
        elif id == 3:
            # Open the gripper
            self.gripper.open()
        pass


class AutoPickTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._im_server = im_server
        # obj_im = InteractiveMarker() ...
        self._im_server.insert(obj_im, feedback_cb = self.handle_feedback)

    def start(self):
        pass

    def handle_feedback(self, feedback):
        pass


def main():
    rospy.init_node('gripper_im_server_node')

    arm = fetch_api.Arm()
    gripper = fetch_api.Gripper()
    im_server = InteractiveMarkerServer('gripper_im_server')
    teleop = GripperTeleop(arm, gripper, im_server)
    #auto_pick = AutoPickTeleop(arm, gripper, im_server)
    teleop.start()
    #auto_pick.start()
    rospy.spin()


if __name__ == "__main__":
    main()
