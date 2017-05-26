#!/usr/bin/env python 

from sensor_msgs.msg import PointCloud2
import perception
import rospy

"""
Starts of node that publishes a saved point cloud 
given a bag while.
"""

def wait_for_time(): 
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

    
def main():                                                                             
    rospy.init_node('publish_saved_cloud')
    wait_for_time()                                                                     
    argv = rospy.myargv()
    if len(argv) < 2:
        print 'Publishes a saved point cloud to a latched topic.'
        print 'Usage: rosrun applications publish_saved_cloud.py ~/cloud.bag'
        return
    path = argv[1]
    camera = perception.MockCamera()
    cloud = camera.read_cloud(path)

    if cloud is None:
        rospy.logerr('Could not load point cloud from {}'.format(path))
        return

    pub = rospy.Publisher('mock_point_cloud', PointCloud2, latch=True, queue_size=1)       
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        cloud.header.stamp = rospy.Time.now()
        # Publish frame id at shelf so that even if robot moves, the point cloud would
        # not move along with it. Publish transform from map to shelf
        cloud.header.frame_id = 'shelf'
        pub.publish(cloud)
        rate.sleep()                                          
    
    
if __name__ == '__main__':
    main()
