#!/usr/bin/env python

import rospy
from multiprocessing import Queue
from shuguru.srv import PutCommand, GetCommand

"""
Starts the command server which accepts a service request
for get and put shoe box command
"""

command_queue = None


def handle_put_command(req):
    global command_queue
    print("Put request received.", req)
    command_queue.put(req)
    return 1


def handle_get_command(req):
    global command_queue
    print("Get request received.", req)
    get = command_queue.get()
    return (get.shoe_id, get.goal_id)


def main():
    rospy.init_node('command_server')

    global command_queue
    command_queue = Queue()

    rospy.Service('put_command', PutCommand, handle_put_command)
    print("Ready to put commands.")
    rospy.Service('get_command', GetCommand, handle_get_command)
    print("Ready to get commands.")

    rospy.spin()


if __name__ == "__main__":
    main()
