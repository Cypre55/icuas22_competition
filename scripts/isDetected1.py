#!/usr/bin/env python

import rospy
import os
from ar_track_alvar_msgs.msg import AlvarMarkers,AlvarMarker
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped,Point

pub1 = rospy.Publisher('/isDetected',Bool,queue_size=10)
pub2 = rospy.Publisher('red/tracker/input_pose', PoseStamped, queue_size=10)
pub3 = rospy.Publisher('/red/tag_position_reconstructed',Point,queue_size=10)

def markerCallback(data):
    global marker_pose
    marker_pose = data

def poseCallback(data):
    global drone_pose
    drone_pose = data
    
def stopper():
    global marker_pose,count,drone_pose
    flag= False
    pubMsg = Bool()
    pubMsg2 = PoseStamped()
    pubMsg3 = Point()
    pubMsg.data = False
    if marker_pose is not None:
        if marker_pose.markers:
            a = marker_pose.markers[0].pose.pose.position.x + drone_pose.pose.pose.position.x
            b = marker_pose.markers[0].pose.pose.position.y + drone_pose.pose.pose.position.y
            c = marker_pose.markers[0].pose.pose.position.z + drone_pose.pose.pose.position.z
            pubMsg.data = True
            flag = True
            pubMsg2.pose.position.x = a
            pubMsg2.pose.position.y = b
            pubMsg2.pose.position.z = c
            pub2.publish(pubMsg2)
            print(a,b,c)
            count = count+1
            print(count)
            if(count>100):
                print("KILLLLLLLLLLL")
                pubMsg3.x = marker_pose.markers[0].pose.pose.position.x
                pubMsg3.y = marker_pose.markers[0].pose.pose.position.y
                pubMsg3.z = marker_pose.markers[0].pose.pose.position.z
                pubMsg2.pose.position.x = 8.5
                pubMsg2.pose.position.y = 0
                pubMsg2.pose.position.z = 2
                pub1.publish(pubMsg)
                pub2.publish(pubMsg2)
                pub3.publish(pubMsg3)
                os.system("rosnode kill /zone3exp")

def isDetected():
    rospy.init_node('isDetected')
    global marker_pose,drone_pose,count
    marker_pose=None
    rospy.Subscriber('/ar_pose_marker',AlvarMarkers,markerCallback)
    rospy.Subscriber('/red/odometry',Odometry,poseCallback)
    count = 0
    while not rospy.is_shutdown():
        stopper()

if __name__ == '__main__':
    try:
        isDetected()
    except rospy.ROSInterruptException:
        pass