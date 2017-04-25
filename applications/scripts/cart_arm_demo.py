#! /usr/bin/env python

import fetch_api
from geometry_msgs.msg import *
from std_msgs.msg import Header
import rospy


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass



def main():

    rospy.init_node('cart_arm_demo')
    wait_for_time()
    argv = rospy.myargv()
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'base_link'
    pose1 = Pose(Point(0.042, 0.384, 1.826), Quaternion(0.173, -0.693, -0.242, 0.657))
    pose2 = Pose(Point(0.047, 0.545, 1.822), Quaternion(-0.274, -0.701, 0.173, 0.635))
    pose1_s = PoseStamped()
    pose1_s.header = header
    pose1_s.pose = pose1
    pose2_s = PoseStamped()
    pose2_s.header = header
    pose2_s.pose = pose2
    gripper_poses = [pose1_s, pose2_s]



    arm = fetch_api.Arm()
    def shutdown():
       arm.cancel_all_goals()
       return
    rospy.on_shutdown(shutdown)
    while(True):
        for pose in gripper_poses:
            rospy.sleep(1)
            error = arm.move_to_pose(pose)
            if error is not None:
                rospy.logerr(error)
       


if __name__ == '__main__':
    main()
