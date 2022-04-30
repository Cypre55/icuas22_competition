#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32

import numpy as np

zone=-1
WAYPOINTS = [
    [[2, -4, 3], [0, 0, -0.682, 0.732]],
    [[9, -4, 3], [0, 0, -0.682, 0.732]],
    [[9, -4, 3], [0, 0, 0, 1]],
    [[9, 4, 3], [0, 0, 0, 1]],
    [[9, 4, 3], [0, 0, 0.682, 0.732]],
    [[2, 4, 3], [0, 0, 0.682, 0.732]]
]
REACH_THRESHOLD = 0.2



def odomCallback(data):
    global drone_pose
    drone_pose = data.pose.pose

def zoneCallback(data):
    global zone
    zone = data.data

def tagCallback(data):
    global tag_detected
    tag_detected = True

def dest_reached(curr_pose, curr_dest):
    curr_pose_arr = np.array([curr_pose.position.x, curr_pose.position.y, curr_pose.position.z])
    curr_dest_arr = np.array([curr_dest[0][0], curr_dest[0][1], curr_dest[0][2]])
    dist = np.linalg.norm(curr_dest_arr - curr_pose_arr)
    if dist < REACH_THRESHOLD:
        return True
    else:
        return False

def move_to_point(x,y,z,rx,ry,rz,rw):
    pubMsg = PoseStamped()
    pubMsg.pose.position.x = x
    pubMsg.pose.position.y = y
    pubMsg.pose.position.z = z

    pubMsg.pose.orientation.x = rx
    pubMsg.pose.orientation.y = ry
    pubMsg.pose.orientation.z = rz
    pubMsg.pose.orientation.w = rw
    pub1.publish(pubMsg)
    rospy.sleep(2.5)

def calculations():
    global CURR_DEST_INDEX
    if zone == 3 and not tag_detected:
        if CURR_DEST_INDEX == -1:
            move_to_point(WAYPOINTS[0][0][0],WAYPOINTS[0][0][1],WAYPOINTS[0][0][2],WAYPOINTS[0][1][0],WAYPOINTS[0][1][1],WAYPOINTS[0][1][2],WAYPOINTS[0][1][3])
            CURR_DEST_INDEX += 1
        if dest_reached(drone_pose, WAYPOINTS[CURR_DEST_INDEX]):
            CURR_DEST_INDEX += 1
            move_to_point(WAYPOINTS[CURR_DEST_INDEX][0][0],WAYPOINTS[CURR_DEST_INDEX][0][1],WAYPOINTS[CURR_DEST_INDEX][0][2],WAYPOINTS[CURR_DEST_INDEX][1][0],WAYPOINTS[CURR_DEST_INDEX][1][1],WAYPOINTS[CURR_DEST_INDEX][1][2],WAYPOINTS[CURR_DEST_INDEX][1][3])



def zone3exp():
    rospy.init_node('zone3exp')
    global drone_pose,marker_pose,pub1, zone, tag_detected, CURR_DEST_INDEX
    CURR_DEST_INDEX = -1
    zone = -1
    tag_detected = False
    drone_pose = None
    rospy.Subscriber('/red/odometry',Odometry,odomCallback)
    rospy.Subscriber('/zone',Int32,zoneCallback)
    rospy.Subscriber('/red/tag_position_reconstructed', Point, tagCallback)
    pub1 = rospy.Publisher('red/tracker/input_pose', PoseStamped, queue_size=10)
    rate =rospy.Rate(10)
    while not rospy.is_shutdown():
        if drone_pose is not None:
            calculations()
            drone_pose = None
        rate.sleep()


if __name__ == '__main__':
    try:
        zone3exp()
    except rospy.ROSInterruptException:
        pass