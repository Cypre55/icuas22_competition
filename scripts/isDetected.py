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
killer = False
def markerCallback(data):
    global marker_pose
    marker_pose = data

# marker_pose.markers[0].pose.pose.position.x

def poseCallback(data):
    # print("posecallback called")
    global drone_pose,header_drone_pose
    drone_pose = data
    header_drone_pose = data.header
    
def stopper():
    global marker_pose,count
    global drone_pose,header_drone_pose,killer
    flag= False
    pubMsg = Bool()
    pubMsg2 = PoseStamped()
    pubMsg3 = Point()
    pubMsg.data = False
    if marker_pose is not None:
        if marker_pose.markers:
            a = marker_pose.markers[0].pose.pose.position.x
            b = marker_pose.markers[0].pose.pose.position.y
            c = marker_pose.markers[0].pose.pose.position.z
            x = drone_pose.pose.pose.orientation.x
            y = drone_pose.pose.pose.orientation.y
            z = drone_pose.pose.pose.orientation.z
            w = drone_pose.pose.pose.orientation.w
            if drone_pose.pose.pose.orientation.z<-0.04:
                b = b+2
            elif drone_pose.pose.pose.orientation.z>0.04:
                b = b-2
            else:
                a = a-2

            flag = True
            pubMsg.data = True
            pubMsg2.pose.position.x = a
            pubMsg2.pose.position.y = b
            pubMsg2.pose.position.z = c
            pubMsg2.pose.orientation.x = x
            pubMsg2.pose.orientation.y = y
            pubMsg2.pose.orientation.z = z
            pubMsg2.pose.orientation.w = w
            pub2.publish(pubMsg2)
            print(marker_pose.markers[0].pose.pose.position.x,marker_pose.markers[0].pose.pose.position.y,marker_pose.markers[0].pose.pose.position.z)
            count = count+1
            print(count)
            if(count>100):
                print(count)
                print("KILLLLLLLLLLL")
                pubMsg3.x = marker_pose.markers[0].pose.pose.position.x
                pubMsg3.y = marker_pose.markers[0].pose.pose.position.y
                pubMsg3.z = marker_pose.markers[0].pose.pose.position.z
                # pubMsg2.pose.position.x = a
                # pubMsg2.pose.position.y = b
                # pubMsg2.pose.position.z = c
                # pubMsg2.pose.orientation.x = x
                # pubMsg2.pose.orientation.y = y
                # pubMsg2.pose.orientation.z = z
                # pubMsg2.pose.orientation.w = w
                # pub2.publish(pubMsg2)
                pub3.publish(pubMsg3)
                os.system("rosnode kill /zone3exp")
                os.system("rosnode kill /isDetected")

    # pub1.publish(pubMsg)
    # if(pubMsg.data):
    # else:
    #     print(marker_pose.markers)
    #     print("ples")

def isDetected():
    rospy.init_node('isDetected')
    global marker_pose
    global drone_pose,header_drone_pose,count
    marker_pose=None
    rospy.Subscriber('/ar_pose_marker',AlvarMarkers,markerCallback)
    rospy.Subscriber('/red/odometry',Odometry,poseCallback)
    count = 0
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        stopper()
        rate.sleep()

if __name__ == '__main__':
    try:
        isDetected()
    except rospy.ROSInterruptException:
        pass