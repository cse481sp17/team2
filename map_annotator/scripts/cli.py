#!/usr/bin/env python
import rospy
from map_annotator.msg import UserAction, PoseNames


POSE_NAMES = 'pose_names'
USER_ACTIONS = 'user_actions'

def help():
    print "Commands"
    print "  list: List saved poses."
    print "  save <name>: Save the robot's current pose as <name>. Overwrites if <name> already exists"
    print "  delete <name>: Delete the pose given by <name>."
    print "  goto <name>: Sends the robot to the pose given by <name>."
    print "  help: Show this list of commands"
    print "  quit: quit the program"
    print ""


class CLIInterface(object):
    def __init__(self):
        self.publisher =  rospy.Publisher('user_actions',UserAction, queue_size = 10) 

        self.poses = []
    
    # Publish msg 
    def save(self, poseName):
        msg = UserAction()
        msg.command = UserAction.SAVE
        msg.name = poseName
        self.publisher.publish(msg) 
        self.poses.append(poseName)
    
    def deletePose(self, poseName):
        if poseName not in self.poses:
            print "No such pose \'" + poseName +"\'"
        else:
            msg = UserAction()
            msg.command = UserAction.DELETE
            msg.name = poseName
            self.publisher.publish(msg) 
    
    def goto(self, poseName):
        if poseName not in self.poses:
            print "No such pose \'" + poseName +"\'"
        else:
            msg = UserAction()
            msg = UserAction()
            msg.command = UserAction.GOTO
            msg.name = poseName
            self.publisher.publish(msg) 
    
    
    def listPoses(self):
        print "Poses:"
        for pose in self.poses:
            print " " + pose
        print ""
    
    def callback(self,new_data):
        self.poses = new_data.poses

if __name__ == "__main__":
    rospy.init_node('cli_publisher')
    
    # Make a publisher to user_actions
    cli = CLIInterface() 

    # Subscribe to pose_names
    rospy.Subscriber("pose_names", PoseNames, cli.callback)
    
    # Start cli
    print "Welcome to the map annotator!"
    help()
    
    while(True):
        cin = raw_input("> ")
        if cin =="quit":
            quit()
        elif cin == "list":
            cli.listPoses()
        elif cin == "help":
            help()
        elif cin.startswith("save"):
            cli.save(cin[4:].strip())
        elif cin.startswith("delete"):
            cli.deletePose(cin[6:].strip())
        elif cin.startswith("goto"):
            cli.goto(cin[4:].strip())
        else:
            print "Unknown cmd type again"
    
     
