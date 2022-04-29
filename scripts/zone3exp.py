#!/usr/bin/env python
import rospy
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge,CvBridgeError
from std_msgs.msg import Int32

zone=-1

def poseCallback(data):
    global drone_pose
    drone_pose = data

def zoneCallback(data):
    global zone
    zone = data.data

def check_error(final_x, current_x):
    error = 0.2
    if(abs(final_x-current_x) < error):
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

def calculations():
    global drone_pose,marker_pose
    if zone==3:
        print("Drone is in zone 3")
        pubMsg = PoseStamped()

        # while(check_error(1.5,drone_pose.pose.pose.position.x) == False):
        pubMsg.pose.position.x = 2
        pubMsg.pose.position.y = -4
        pubMsg.pose.position.z = 3

        pubMsg.pose.orientation.x = 0
        pubMsg.pose.orientation.y = 0
        pubMsg.pose.orientation.z = -0.682
        pubMsg.pose.orientation.w = 0.7352

        pub1.publish(pubMsg)
        rospy.sleep(5)
        # move_to_point(1.5,-4,3,0,0,-0.682,0.7)
        # while(check_error(9,drone_pose.pose.pose.position.x) == False):
        pubMsg.pose.position.x = 9
        pubMsg.pose.position.y = -4
        pubMsg.pose.position.z = 3

        pubMsg.pose.orientation.x = 0
        pubMsg.pose.orientation.y = 0
        pubMsg.pose.orientation.z = -0.682
        pubMsg.pose.orientation.w = 0.7352

        pub1.publish(pubMsg)
        rospy.sleep(5)
        pubMsg.pose.position.x = 9
        pubMsg.pose.position.y = -4
        pubMsg.pose.position.z = 3

        pubMsg.pose.orientation.x = 0
        pubMsg.pose.orientation.y = 0
        pubMsg.pose.orientation.z = 0
        pubMsg.pose.orientation.w = 1.0

        pub1.publish(pubMsg)
        rospy.sleep(2)
        # while(check_error(9,drone_pose.pose.pose.position.x) == False):
        pubMsg.pose.position.x = 9
        pubMsg.pose.position.y = 4
        pubMsg.pose.position.z = 3

        pubMsg.pose.orientation.x = 0
        pubMsg.pose.orientation.y = 0
        pubMsg.pose.orientation.z = 0
        pubMsg.pose.orientation.w = 1.0

        pub1.publish(pubMsg)
        rospy.sleep(5)

        pubMsg.pose.position.x = 9
        pubMsg.pose.position.y = 4
        pubMsg.pose.position.z = 3

        pubMsg.pose.orientation.x = 0
        pubMsg.pose.orientation.y = 0
        pubMsg.pose.orientation.z = 0.687
        pubMsg.pose.orientation.w = 0.732

        pub1.publish(pubMsg)
        rospy.sleep(2)


        rospy.sleep(5)
        # while(check_error(1.5,drone_pose.pose.pose.position.x) == False):
        pubMsg.pose.position.x = 2
        pubMsg.pose.position.y = 4
        
        pubMsg.pose.position.z = 3

        pubMsg.pose.orientation.x = 0
        pubMsg.pose.orientation.y = 0
        pubMsg.pose.orientation.z = 0.68
        pubMsg.pose.orientation.w = 0.732
        pub1.publish(pubMsg)        
        rospy.sleep(5)    


def zone3exp():
    rospy.init_node('zone3exp')
    global drone_pose,marker_pose,pub1
    drone_pose = None
    rospy.Subscriber('/red/odometry',Odometry,poseCallback)
    rospy.Subscriber('/zone',Int32,zoneCallback)
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