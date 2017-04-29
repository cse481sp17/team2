#!/usr/bin/env python

import rospy
import numpy as np
from interactive_markers.interactive_marker_server import *
from geometry_msgs.msg import *
from visualization_msgs.msg import *
import fetch_api
import tf
from std_msgs.msg import *

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
        self.gripper_im = InteractiveMarker()
        self.gripper_im.header.frame_id = 'base_link'
        self.gripper_im.name = GRIPPER_MARKER_NAME
        self.gripper_im.description = GRIPPER_MARKER_NAME
        self.gripper_im.scale = .25
        
        # Initial IM Pose
        pose = Pose()
        pose.position.x = 1
        pose.position.y = 0
        pose.position.z = 1
        pose.orientation.w = 1
        self.gripper_im.pose = pose

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
        self.gripper_im.controls.append(gripper_control)

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

        self.gripper_im.menu_entries.append(move_menu_entry)
        self.gripper_im.menu_entries.append(close_menu_entry)
        self.gripper_im.menu_entries.append(open_menu_entry)

        addControls(self.gripper_im)

        self._im_server.insert(self.gripper_im, feedback_cb = self.handle_feedback)
        
        self._im_server.applyChanges()

    def start(self):
        pass

    def handle_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            id = feedback.menu_entry_id
            if id == 1:  
                kwargs = {
                    'allowed_planning_time': 20,
                    'execution_timeout': 40,
                    'num_planning_attempts': 30,
                    'replan': False,
                }
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = "base_link"
                pose_stamped.pose = feedback.pose
                                
                self.arm.move_to_pose(pose_stamped, **kwargs)
            elif id == 2:
                # Close the gripper
                self.gripper.close()
            elif id == 3:
                # Open the gripper
                self.gripper.open()
        # TODO:Need to update pose locations 
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            i_markers = self.gripper_im.controls
            poseStamped = PoseStamped()
            poseStamped.header = feedback.header
            poseStamped.pose = feedback.pose
            ik = self.arm.compute_ik(poseStamped)
            
            color = ColorRGBA()
            i_marker = self._im_server.get(feedback.marker_name)
            if ik:
                color.r = 0.0
                color.g = 1.0
                color.a = 1.0
            else:
                color.r = 1.0
                color.g = 0.0
                color.a = 1.0
            for marker in i_marker.controls[0].markers:
                marker.color = color
            self._im_server.insert(i_marker)
            self._im_server.applyChanges()

class AutoPickTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._im_server = im_server
        self.gripper = gripper
        self.arm = arm
        self.pregrasp_offset = -.1
        self.lift_offset = .2


        # InteractiveMarker - top level object
        self.obj_im = InteractiveMarker()
        self.obj_im.header.frame_id = 'base_link'
        self.obj_im.name = GRIPPER_MARKER_NAME
        self.obj_im.description = GRIPPER_MARKER_NAME
        self.obj_im.scale = .25

        # Initial IM Pose
        pose = Pose()
        pose.position.x = 1
        pose.position.y = 0
        pose.position.z = 1
        pose.orientation.w = 1
        self.obj_im.pose = pose

        # Markers for gripper and fingers
        gripper_marker = Marker()
        gripper_marker.id = 1
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
        l_finger_m.id = 2
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
        r_finger_m.id = 3
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

        # Markers for gripper and fingers
        pregrasp_gripper_marker = Marker()
        pregrasp_gripper_marker.id = 4
        pregrasp_gripper_marker.type = Marker.MESH_RESOURCE
        pregrasp_gripper_marker.mesh_resource = GRIPPER_MESH
        pregrasp_gripper_marker.pose.orientation.w = 1
        pregrasp_gripper_marker.pose.position.x = .166 + self.pregrasp_offset
        pregrasp_gripper_marker.scale.x = 1
        pregrasp_gripper_marker.scale.y = 1
        pregrasp_gripper_marker.scale.z = 1
        pregrasp_gripper_marker.color.r = 1.0
        pregrasp_gripper_marker.color.a = 1.0

        pregrasp_l_finger_m = Marker()
        pregrasp_l_finger_m.id = 5
        pregrasp_l_finger_m.type = Marker.MESH_RESOURCE
        pregrasp_l_finger_m.mesh_resource = L_FINGER_MESH
        pregrasp_l_finger_m.pose.orientation.w = 1
        pregrasp_l_finger_m.pose.position.y = -.06
        pregrasp_l_finger_m.pose.position.x = .166 + self.pregrasp_offset
        pregrasp_l_finger_m.scale.x = 1
        pregrasp_l_finger_m.scale.y = 1
        pregrasp_l_finger_m.scale.z = 1
        pregrasp_l_finger_m.color.r = 1.0
        pregrasp_l_finger_m.color.a = 1.0
               
        pregrasp_r_finger_m = Marker()
        pregrasp_r_finger_m.id = 6
        pregrasp_r_finger_m.type = Marker.MESH_RESOURCE
        pregrasp_r_finger_m.mesh_resource = R_FINGER_MESH
        pregrasp_r_finger_m.pose.orientation.w = 1
        pregrasp_r_finger_m.pose.position.y = .06
        pregrasp_r_finger_m.pose.position.x = .166 + self.pregrasp_offset
        pregrasp_r_finger_m.scale.x = 1
        pregrasp_r_finger_m.scale.y = 1
        pregrasp_r_finger_m.scale.z = 1
        pregrasp_r_finger_m.color.r = 1.0
        pregrasp_r_finger_m.color.a = 1.0

        # Markers for gripper and fingers
        lift_gripper_marker = Marker()
        lift_gripper_marker.id = 7
        lift_gripper_marker.type = Marker.MESH_RESOURCE
        lift_gripper_marker.mesh_resource = GRIPPER_MESH
        lift_gripper_marker.pose.orientation.w = 1
        lift_gripper_marker.pose.position.x = .166
        lift_gripper_marker.pose.position.z = self.lift_offset
        lift_gripper_marker.scale.x = 1
        lift_gripper_marker.scale.y = 1
        lift_gripper_marker.scale.z = 1
        lift_gripper_marker.color.r = 1.0
        lift_gripper_marker.color.a = 1.0

        lift_l_finger_m = Marker()
        lift_l_finger_m.id = 8
        lift_l_finger_m.type = Marker.MESH_RESOURCE
        lift_l_finger_m.mesh_resource = L_FINGER_MESH
        lift_l_finger_m.pose.orientation.w = 1
        lift_l_finger_m.pose.position.y = -.06
        lift_l_finger_m.pose.position.x = .166
        lift_l_finger_m.pose.position.z = self.lift_offset
        lift_l_finger_m.scale.x = 1
        lift_l_finger_m.scale.y = 1
        lift_l_finger_m.scale.z = 1
        lift_l_finger_m.color.r = 1.0
        lift_l_finger_m.color.a = 1.0
               
        lift_r_finger_m = Marker()
        lift_r_finger_m.id = 9
        lift_r_finger_m.type = Marker.MESH_RESOURCE
        lift_r_finger_m.mesh_resource = R_FINGER_MESH
        lift_r_finger_m.pose.orientation.w = 1
        lift_r_finger_m.pose.position.y = .06
        lift_r_finger_m.pose.position.x = .166
        lift_r_finger_m.pose.position.z = self.lift_offset
        lift_r_finger_m.scale.x = 1
        lift_r_finger_m.scale.y = 1
        lift_r_finger_m.scale.z = 1
        lift_r_finger_m.color.r = 1.0
        lift_r_finger_m.color.a = 1.0

        grasped_obj_marker = Marker()
        grasped_obj_marker.id = 20
        grasped_obj_marker.type = Marker.CUBE
        grasped_obj_marker.pose.position.x = .166
        grasped_obj_marker.scale.x = .06
        grasped_obj_marker.scale.y = .06
        grasped_obj_marker.scale.z = .06
        grasped_obj_marker.color.b = 1.0
        grasped_obj_marker.color.a = 1.0

        # This control is the dropdown menu
        gripper_control = InteractiveMarkerControl()
        gripper_control.orientation.w = 1
        gripper_control.interaction_mode = InteractiveMarkerControl.MENU
        
        # Attach the gripper to the menu
        gripper_control.markers.append(grasped_obj_marker)

        gripper_control.markers.append(gripper_marker)
        gripper_control.markers.append(l_finger_m)
        gripper_control.markers.append(r_finger_m)

        gripper_control.markers.append(pregrasp_gripper_marker)
        gripper_control.markers.append(pregrasp_l_finger_m)
        gripper_control.markers.append(pregrasp_r_finger_m)

        gripper_control.markers.append(lift_gripper_marker)
        gripper_control.markers.append(lift_l_finger_m)
        gripper_control.markers.append(lift_r_finger_m)

        gripper_control.always_visible = True
        self.obj_im.controls.append(gripper_control)

        # Menu Entries
        pregrasp_menu_entry = MenuEntry()
        pregrasp_menu_entry.id = 1
        pregrasp_menu_entry.title = "Pregrasp"

        grasp_menu_entry = MenuEntry()
        grasp_menu_entry.id = 2
        grasp_menu_entry.title = "Grasp"

        lift_menu_entry = MenuEntry()
        lift_menu_entry.id = 3
        lift_menu_entry.title = "Lift"

        close_menu_entry = MenuEntry()
        close_menu_entry.id = 4
        close_menu_entry.title = "Close"

        open_menu_entry = MenuEntry()
        open_menu_entry.id = 5
        open_menu_entry.title = "Open"


        self.obj_im.menu_entries.append(pregrasp_menu_entry)
        self.obj_im.menu_entries.append(grasp_menu_entry)
        self.obj_im.menu_entries.append(lift_menu_entry)
        self.obj_im.menu_entries.append(close_menu_entry)
        self.obj_im.menu_entries.append(open_menu_entry)

        addControls(self.obj_im)

        self._im_server.insert(self.obj_im, feedback_cb = self.handle_feedback)

        self._im_server.applyChanges()

    def start(self):
        pass

    def handle_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            id = feedback.menu_entry_id
            kwargs = {
                    'allowed_planning_time': 20,
                    'execution_timeout': 40,
                    'num_planning_attempts': 30,
                    'replan': False,
                }
            # Pregrasp
            if id == 1:  
                # Transform pose 

                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = "base_link"
                pose_stamped.pose = transform(feedback.pose, id)
                                
                self.arm.move_to_pose(pose_stamped, **kwargs)
            # Grasp
            elif id == 2:  

                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = "base_link"
                pose_stamped.pose = feedback.pose
                                
                self.arm.move_to_pose(pose_stamped, **kwargs)
                
            # Lift
            elif id == 3: 
                self.gripper.close()
                # Transform pose 

                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = "base_link"


                pose_stamped.pose = transform(feedback.pose, id)
                                
                self.arm.move_to_pose(pose_stamped, **kwargs)
            elif id == 4:
                # Close the gripper
                self.gripper.close()
            elif id == 5:
                # Open the gripper
                self.gripper.open()
        # TODO:Need to update pose locations 
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            i_markers = self.obj_im.controls
            grasp_poseStamped = PoseStamped()
            grasp_poseStamped.header = feedback.header
            grasp_poseStamped.pose = feedback.pose

            pregrasp_poseStamped = PoseStamped()
            pregrasp_poseStamped.header = feedback.header
            pregrasp_poseStamped.pose = transform(feedback.pose, 1)

            lift_poseStamped = PoseStamped()
            lift_poseStamped.header = feedback.header
            lift_poseStamped.pose = transform(feedback.pose, 3)

            ik_grasp = self.arm.compute_ik(grasp_poseStamped)
            ik_pregrasp = self.arm.compute_ik(pregrasp_poseStamped)
            ik_lift = self.arm.compute_ik(lift_poseStamped)
            
            grasp_color = ColorRGBA()
            pregrasp_color = ColorRGBA()
            lift_color = ColorRGBA()
            i_marker = self._im_server.get(feedback.marker_name)
            if ik_grasp:
                grasp_color.r = 0.0
                grasp_color.g = 1.0
                grasp_color.a = 1.0
            else:
                grasp_color.r = 1.0
                grasp_color.g = 0.0
                grasp_color.a = 1.0
            for marker in i_marker.controls[0].markers:
                if marker.id < 4:
                    marker.color = grasp_color

            if ik_pregrasp:
                pregrasp_color.r = 0.0
                pregrasp_color.g = 1.0
                pregrasp_color.a = 1.0
            else:
                pregrasp_color.r = 1.0
                pregrasp_color.g = 0.0
                pregrasp_color.a = 1.0
            for marker in i_marker.controls[0].markers:
                if marker.id > 3 and marker.id < 6:
                    marker.color = pregrasp_color

            if ik_lift:
                lift_color.r = 0.0
                lift_color.g = 1.0
                lift_color.a = 1.0
            else:
                lift_color.r = 1.0
                lift_color.g = 0.0
                lift_color.a = 1.0
            for marker in i_marker.controls[0].markers:
                if marker.id > 6 and marker.id < 10:
                    marker.color = lift_color

            self._im_server.insert(i_marker)
            self._im_server.applyChanges()

def transform(pose, id):

    orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)

    rot_matrix = tf.transformations.quaternion_matrix(orientation_tuple)
    rot_matrix[0][3] = pose.position.x
    rot_matrix[1][3] = pose.position.y
    rot_matrix[2][3] = pose.position.z
    pregrasp = np.zeros(shape=(4,4), dtype=float)

    if id == 1:
        pregrasp[0][3] = -.1
    elif id == 3:
        pregrasp[2][3] = .2
    pregrasp[3][3] = 1.0
    pregrasp[0][0] = 1.0
    pregrasp[1][1] = 1.0
    pregrasp[2][2] = 1.0  
    ans = np.dot(rot_matrix, pregrasp)
    quaternion = tf.transformations.quaternion_from_matrix(ans)

    pose = Pose()
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    pose.position.x = ans[0][3]
    pose.position.y = ans[1][3]
    pose.position.z = ans[2][3]

    return pose

def addControls(im):
    # Controls
    # Axis shift controls
    x_axis_control = InteractiveMarkerControl()
    x_axis_control.orientation.w = 1
    x_axis_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    x_axis_control.always_visible = True
    im.controls.append(x_axis_control)
    y_axis_control = InteractiveMarkerControl()
    y_axis_control.orientation.w = 1
    y_axis_control.orientation.z = 1
    y_axis_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    y_axis_control.always_visible = True

    im.controls.append(y_axis_control)

    z_axis_control = InteractiveMarkerControl()
    z_axis_control.orientation.w = 1
    z_axis_control.orientation.y = 1
    z_axis_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    z_axis_control.always_visible = True

    im.controls.append(z_axis_control)

    # Rotation Controls
    z_axis_rotate = InteractiveMarkerControl()
    z_axis_rotate.orientation.w = 1
    z_axis_rotate.orientation.y = 1
    z_axis_rotate.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    z_axis_rotate.always_visible = True

    im.controls.append(z_axis_rotate) 
    
    x_axis_rotate = InteractiveMarkerControl()
    x_axis_rotate.orientation.w = 1
    x_axis_rotate.orientation.z = 1
    x_axis_rotate.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    x_axis_rotate.always_visible = True
    im.controls.append(x_axis_rotate)
    
    y_axis_rotate = InteractiveMarkerControl()
    y_axis_rotate.orientation.w = 1
    y_axis_rotate.orientation.x = 1
    y_axis_rotate.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    y_axis_rotate.always_visible = True
    im.controls.append(y_axis_rotate)   

def main():
    rospy.init_node('gripper_im_server_node')

    arm = fetch_api.Arm()
    gripper = fetch_api.Gripper()
    im_server = InteractiveMarkerServer('gripper_im_server', q_size=2)
    auto_pick_server = InteractiveMarkerServer('auto_pick_im_server', q_size=2)
    teleop = GripperTeleop(arm, gripper, im_server)
    auto_pick = AutoPickTeleop(arm, gripper, auto_pick_server)
    teleop.start()
    auto_pick.start()
    rospy.spin()


if __name__ == "__main__":
    main()
