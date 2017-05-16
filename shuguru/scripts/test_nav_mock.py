#!/usr/bin/env python

import rospy
from shuguru.srv import PutCommand

def main():
    rospy.init_node('test_nav_mock')
    rospy.wait_for_service('put_command')
    put_command = rospy.ServiceProxy('put_command', PutCommand)
    
    put_command(1,1)

if __name__ == '__main__':
    main()
