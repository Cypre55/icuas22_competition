#!/usr/bin/env python
import rospy
import os
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped,Point

pub1 = rospy.Publisher('/isDetected',Bool,queue_size=10)
pub2 = rospy.Publisher('red/tracker/input_pose', PoseStamped, queue_size=10)
pub3 = rospy.Publisher('/red/tag_position_reconstructed', Point, queue_size=10)
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
    global drone_pose,header_drone_pose
    flag= False
    pubMsg = Bool()
    pubMsg2 = PoseStamped()
    pubMsg3 = Point()
    pubMsg.data = False
    if marker_pose is not None:
        if marker_pose.markers:
            # a = marker_pose.markers[0].pose.pose.position.x + drone_pose.pose.pose.position.x
            # b = marker_pose.markers[0].pose.pose.position.y + drone_pose.pose.pose.position.y
            # c = marker_pose.markers[0].pose.pose.position.z + drone_pose.pose.pose.position.z
            a = marker_pose.markers[0].pose.pose.position.x
            b = marker_pose.markers[0].pose.pose.position.y
            c = marker_pose.markers[0].pose.pose.position.z
            if not flag:
                print(a,b,c)
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
                print(count)
                print("KILLLLLLLLLLL")
                pubMsg2.pose.position.x = 8.5
                pubMsg2.pose.position.y = -3
                pubMsg2.pose.position.z = 2
                pub2.publish(pubMsg2)
                pubMsg3.x = a
                pubMsg3.y = b
                pubMsg3.z = c
                pub3.publish(pubMsg3)
                os.system("rosnode kill /zone3exp")
                # os.system("rosnode kill /isDetected")
                pub1.publish(pubMsg)

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

if __name__ == '__main__':
    try:
        isDetected()
    except rospy.ROSInterruptException:
        pass