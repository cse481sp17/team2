#!/usr/bin/env python

import pickle
import rospy
import tf
import fetch_api
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers

def help():
    print "Commands"
    print "  list: List sequence of actions."
    print "  delete <action index>: hoge"
    print "  create <action type>: huga"
    print "  save <file name>: hage"
    print "  load <file name>: hage"
    print "  execute: foo"
    print "  help: Show this list of commands"
    print "  quit: Quit the program"
    print ""


class ActionType(object):
    def __init__(self, action, arId = 0, arPose=None, wristPose=None ):
        self.arId = arId
        self.action = action
        self.arPose = arPose
        self.wristPose = wristPose


class ActionSaver(object):

    MOVE=0
    OPEN=1
    CLOSE=2

    AR_POSE = "/ar_pose_marker"

    def __init__(self):
        self.actions = []
        self.markers = []
        self.ar_sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.arCallback)
        self.listener = tf.TransformListener()
    
    def save(self, fileName):
        if len(fileName) == 0:
            print("Invalid Name: Empty string")
            return
        pickle.dump(self.actions, open(fileName, 'wb'))

    def load(self, fileName):
        if len(fileName) == 0:
            print("Invalid Name: Empty string")
            return
        try:
            self.actions = pickle.load(open(fileName, 'rb'))
        except:
            self.actions = []
    
    def delete(self, actionIdx):
        if actionIdx >= len(self.actions):
            print("Index out of bounds")
        else:
            del self.actions[actionIdx]
    
    def create(self, actionType):
        if actionType == "move":
            # Save wrist_roll's pose in base_link frame
            (trans, rot) = self.listener.lookupTransform('base_link', '/wrist_roll_link', rospy.Time(0))
            wristPose = Pose()
            wristPose.position.x = trans[0]
            wristPose.position.y = trans[1]
            wristPose.position.z = trans[2]
            wristPose.orientation.x = rot[0]
            wristPose.orientation.y = rot[1]
            wristPose.orientation.z = rot[2]
            wristPose.orientation.w = rot[3]
            if len(self.markers) == 0:
                print("There's no marker")
                return
            # Save ar_marker's pose in base_link frame
            print("Relative to which marker do you want to save this pose?")
            for i, marker in enumerate(self.markers):
                print(i)
                print(marker.pose.pose.position)
                print("")
            cin = raw_input("> ")
            indexIn = int(cin)
            arPose = self.markers[indexIn].pose.pose
            # Create Action
            action = ActionType(ActionSaver.MOVE, marker.id, arPose, wristPose)
            self.actions.append(action)
        elif actionType == "open":
            action = ActionType(ActionSaver.OPEN)
            self.actions.append(action)
        elif actionType == "close":
            action = ActionType(ActionSaver.CLOSE)
            self.actions.append(action)
        else:
            print "No such action type"

    def list(self):
        for i, action in enumerate(self.actions):
            if action.action == ActionSaver.MOVE:
                print(i, "MOVE")
            elif action.action == ActionSaver.OPEN:
                print(i, "OPEN")
            elif action.action == ActionSaver.CLOSE:
                print(i, "CLOSE")

    def execute(self):
        arm = fetch_api.Arm()
        gripper = fetch_api.Gripper()
        for action in self.actions:
            if action.action == ActionSaver.MOVE:
                # Compute the pose of the wrist in base_link
                ar = fetch_api.pose2matrix(action.arPose)

                ar2wrist = fetch_api.pose2transform(action.arPose, action.wristPose, True)
                
                # Save the ar marker's id and load it
                wrist = np.dot(fetch_api.pose2matrix(filter(lambda x: x.id == action.arId, self.markers)[0].pose.pose), ar2wrist)
                # Navigate the arm there
                kwargs = {
                    'allowed_planning_time': 20,
                    'execution_timeout': 40,
                    'num_planning_attempts': 30,
                    'replan': False,
                }
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = "base_link"
                pose_stamped.pose = fetch_api.matrix2pose(wrist)
                arm.move_to_pose(pose_stamped, **kwargs)
                # Wait?

            elif action.action == ActionSaver.OPEN:
                gripper.open()
            elif action.action == ActionSaver.CLOSE:
                gripper.close()

    def arCallback(self, msg):
        markers = msg.markers
        markers.sort(key=lambda x: x.pose.pose.position.x)
        if len(markers) < len(self.markers):
            return
        self.markers = markers


if __name__ == "__main__":
    rospy.init_node('action_saver')
    
    action_saver = ActionSaver() 
    
    # Start action_saver
    print "Welcome to the action saver!"
    help()
    
    r = rospy.Rate(10)
    while(True):
        cin = raw_input("> ")
        if cin =="quit":
            quit()
        elif cin == "help":
            help()
        elif cin == "list":
            action_saver.list()
        elif cin.startswith("create"):
            action_saver.create(cin[6:].strip())
        elif cin.startswith("delete"):
            action_saver.delete(int(cin[6:].strip()))
        elif cin.startswith("save"):
            action_saver.save(cin[4:].strip())
        elif cin.startswith("load"):
            action_saver.load(cin[4:].strip())
        elif cin.startswith("execute"):
            action_saver.execute()
        else:
            print "Unknown cmd type again"
        r.sleep()
