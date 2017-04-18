#!/usr/bin/env python

import fetch_api
import rospy
import math
import tf

from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
import sys, select, termios, tty

msg = """
Control Your Fetch!
---------------------------
Moving around:
        w
   a    s    d

Space: force stop
i/k: increase/decrease only linear speed by 5 cm/s
u/j: increase/decrease only angular speed by 0.25 rads/s
anything else: stop smoothly

CTRL-C to quit
"""

moveBindings = {'w': (1, 0), 'a': (0, 1), 'd': (0, -1), 's': (-1, 0)}

speedBindings = {
    'i': (0.05, 0),
    'k': (-0.05, 0),
    'u': (0, 0.25),
    'j': (0, -0.25),
}


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


speed = .2
turn = 1
next_id = 0

marker_1 = (2,0,0)
marker_2 = (3,-2,0)
marker_3 = (4,-1,0)

current_pos = 0
current_target = None
# Moving false when we are doing a keyboard teleop
#        true when fetch is moving towards a goal
moving = False

last_pos = Point(10000000,0,0)
EPSILON = 0.1

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)

def show_sphere_in_rviz(input_id):
    marker = Marker(
                type=Marker.SPHERE,
                id=input_id,
                lifetime=rospy.Duration(0),
                pose=Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.06, 0.06, 0.06),
                header=Header(frame_id='base_link'),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8))
    marker_publisher.publish(marker)


def within_epsilon(pose_1, pose_2):
    global EPSILON
    dsq = ((pose_1.x-pose_2.x)*(pose_1.x-pose_2.x))+((pose_1.y-pose_2.y)*(pose_1.y-pose_2.y))+((pose_1.z-pose_2.z)*(pose_1.z-pose_2.z))
    return dsq < EPSILON

def path_callback(data):
    global next_id
    global last_pos
    global current_pos
    next_id = next_id+1
    thispos = data.pose.pose.position
    current_pos = data.pose.pose
    if not within_epsilon(thispos, last_pos):
        last_pos = thispos
        show_sphere_in_rviz(next_id)

def handle_viz_input(input):
    global current_target
    global moving
    if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
        rospy.loginfo(input.marker_name + ' was clicked.\n\r')
        if input.marker_name == 'Marker 1':
            x = marker_1[0]
            y = marker_1[1]
            z = marker_1[2]
        elif input.marker_name == 'Marker 2':
            x = marker_2[0]
            y = marker_2[1]
            z = marker_2[2]
        elif input.marker_name == 'Marker 3':
            x = marker_3[0]
            y = marker_3[1]
            z = marker_3[2]
        # Set the destination
        moving = True
        current_target = Point(x, y, z)

# Creates markers and adds them to rviz
def make_marker(server, marker_name,position):
    # Create an Interactive Marker:
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "odom"
    int_marker.name = marker_name

    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.scale.x = 0.45
    box_marker.scale.y = 0.45
    box_marker.scale.z = 0.45
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 1.0
    box_marker.pose.position = position
    
    # Marker Control, add marker
    button_control = InteractiveMarkerControl()
    button_control.interaction_mode = InteractiveMarkerControl.BUTTON
    button_control.always_visible = True
    button_control.markers.append(box_marker)
    int_marker.controls.append( button_control )

    server.insert(int_marker, handle_viz_input)
    server.applyChanges()

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    print(dir(Quaternion))
    rospy.init_node('fetch_teleop_key')
    base = fetch_api.Base()

    x = 0
    th = 0
    status = 0
    count = 0
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0
    try:
        print msg
        print vels(speed, turn)
        marker_publisher = rospy.Publisher('visualization_marker', Marker)
        server = InteractiveMarkerServer("simple_marker")
        rospy.Subscriber("odom", Odometry, path_callback)

        pose = Point(marker_1[0], marker_1[1], marker_1[2])
        make_marker(server, "Marker 1", pose)
        
        pose = Point(marker_2[0], marker_2[1], marker_2[2])
        make_marker(server, "Marker 2", pose)
        
        pose = Point(marker_3[0], marker_3[1], marker_3[2])
        make_marker(server, "Marker 3", pose)
        
       
        global moving 
        global current_pos
        global current_target
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if moving:
                x_diff = current_pos.position.x - current_target.x
                y_diff = current_pos.position.y - current_target.y
                quaternion = (current_pos.orientation.x, current_pos.orientation.y,
                              current_pos.orientation.z, current_pos.orientation.w)
                euler = tf.transformations.euler_from_quaternion(quaternion)
                yaw = euler[2]
                goal = math.atan2(y_diff, x_diff) + math.pi
                if goal > math.pi:
                    goal -= 2*math.pi
                rot_diff = yaw - goal

                if abs(rot_diff) < 0.1:
                    control_speed = 0.3
                    control_turn = 0.0
                else:
                    if rot_diff < 0:
                        control_turn = 0.3
                    else:
                        control_turn = -0.3
                    control_speed = 0.0

                if within_epsilon(current_pos.position, current_target):
                    moving = False
                    control_speed = 0.0
                    control_turn = 0.0

            else:
                key = getKey()
                if key in moveBindings.keys():
                    x = moveBindings[key][0]
                    th = moveBindings[key][1]
                    count = 0
                elif key in speedBindings.keys():
                    speed += speedBindings[key][0]
                    turn += speedBindings[key][1]
                    count = 0

                    print vels(speed, turn)
                    if (status == 14):
                        print msg
                    status = (status + 1) % 15
                elif key == ' ':
                    x = 0
                    th = 0
                    control_speed = 0
                    control_turn = 0
                else:
                    count = count + 1
                    if count > 4:
                        x = 0
                        th = 0
                    if (key == '\x03'):
                        break

                target_speed = speed * x
                target_turn = turn * th

                if target_speed > control_speed:
                    control_speed = min(target_speed, control_speed + 0.02)
                elif target_speed < control_speed:
                    control_speed = max(target_speed, control_speed - 0.02)
                else:
                    control_speed = target_speed

                if target_turn > control_turn:
                    control_turn = min(target_turn, control_turn + 0.1)
                elif target_turn < control_turn:
                    control_turn = max(target_turn, control_turn - 0.1)
                else:
                    control_turn = target_turn

            base.move(control_speed, control_turn)
            r.sleep()
    except e:
        rospy.logerr('{}'.format(e))
    finally:
        base.stop()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
