#!/usr/bin/env python
from itertools import count
import rospy,os
from geometry_msgs.msg import PoseStamped, Point, Vector3, Quaternion, Twist, Transform
from tf.transformations import euler_from_quaternion, quaternion_from_euler 
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from nav_msgs.msg import Odometry
from ar_track_alvar_msgs.msg import AlvarMarkers,AlvarMarker
from std_msgs.msg import Int32
import numpy as np
from sensor_msgs.msg import Image

zone = -1
WAYPOINTS = []

def waypoints_generation():
    iterations = 20
    z = 3
    # right wall
    for i in range(iterations):
        WAYPOINTS.append([[2+(9-2)*i/(iterations-1),-4,z],[0,0,-0.707,0.707]])
    # yaw
    iterations = 7
    for i in range(iterations):
        WAYPOINTS.append([[9,-4,z],[0,0,-0.707+(0.707)*i/(iterations-1),0.707+(1-0.707)*i/(iterations-1)]])
    # front wall
    iterations = 10
    for i in range(iterations):
        WAYPOINTS.append([[9, -4+(7)*i/(iterations-1), z], [0, 0, 0, 1]])
    # yaw
    iterations = 7
    for i in range(iterations):
        WAYPOINTS.append([[9,4,z],[0,0,(0.707)*i/(iterations-1),1+(0.707-1)*i/(iterations-1)]])
    # left wall
    iterations = 20
    for i in range(iterations):
        WAYPOINTS.append([[9+(2-9)*i/(iterations-1), 4, z], [0, 0, 0.707, 0.707]])

REACH_THRESHOLD = 0.4

pub1 = rospy.Publisher('red/tracker/input_pose', PoseStamped, queue_size=10)
pub2 = rospy.Publisher('/red/tracker/input_trajectory',MultiDOFJointTrajectory,queue_size=10)
pub3 = rospy.Publisher('/red/tag_position_reconstructed',Point,queue_size=10)
pub4 = rospy.Publisher('/red/tag_image_annotated',Image,queue_size=10)
detected = False
reached_last_point = False

def reach_point(x,y):
    global drone_pose,REACH_THRESHOLD
    if(abs(drone_pose.pose.pose.position.x - x) < REACH_THRESHOLD and abs(drone_pose.pose.pose.position.y - y) < REACH_THRESHOLD):
        return True
    else:
        return False

def annotatedCallback(data):
    global tag_annotated
    tag_annotated = data

def markerCallback(data):
    global marker_pose
    marker_pose = data

def odomCallback(data):
    global drone_pose
    drone_pose = data

def zoneCallback(data):
    global zone
    zone = data.data

def regenerate_callback(event):
    exploration_trajectory()

def reachPointCallback(event):
    global reached_last_point,drone_pose
    if drone_pose is not None:
        reached_last_point = reach_point(WAYPOINTS[len(WAYPOINTS)-1][0][0],WAYPOINTS[len(WAYPOINTS)-1][0][1])

def exploration_trajectory():
    traj = MultiDOFJointTrajectory()

    V = Twist(linear=Vector3( 0, 0, 0), angular=Vector3( 0, 0, 0))
    A = Twist(linear=Vector3( 0, 0, 0), angular=Vector3( 0, 0, 0))
    for i in range(len(WAYPOINTS)):
        Pi = Vector3(WAYPOINTS[i][0][0],WAYPOINTS[i][0][1],WAYPOINTS[i][0][2])
        Qi = Quaternion(WAYPOINTS[i][1][0],WAYPOINTS[i][1][1],WAYPOINTS[i][1][2],WAYPOINTS[i][1][3])
        Ti = Transform(translation = Pi, rotation = Qi)
        trajPi = MultiDOFJointTrajectoryPoint(transforms = [Ti], velocities = [V], accelerations = [A])
        traj.points.append(trajPi)
    pub2.publish(traj)
    print("WAY PUB")

def detection():
    global marker_pose,drone_pose, count,detected,tag_annotated
    pubMsg1 = PoseStamped()
    pubMsg3 = Point()
    pubMsg4 = Image()
    traj = MultiDOFJointTrajectory()
    
    if marker_pose is not None:
        if marker_pose.markers:
            print("Detected")
            detected = True
            a = marker_pose.markers[0].pose.pose.position.x
            b = marker_pose.markers[0].pose.pose.position.y
            c = marker_pose.markers[0].pose.pose.position.z
            print(a,b,c)
            lx,ly,lz = drone_pose.pose.pose.position.x,drone_pose.pose.pose.position.y,drone_pose.pose.pose.position.z
            qx,qy,qz,qw = drone_pose.pose.pose.orientation.x,drone_pose.pose.pose.orientation.y,drone_pose.pose.pose.orientation.z,drone_pose.pose.pose.orientation.w
            pubMsg1.pose.position.x = lx
            pubMsg1.pose.position.y = ly
            pubMsg1.pose.position.z = lz
            pubMsg1.pose.orientation.x = qx
            pubMsg1.pose.orientation.y = qy
            pubMsg1.pose.orientation.z = qz
            pubMsg1.pose.orientation.w = qw
            P0 = Vector3(drone_pose.pose.pose.position.x,drone_pose.pose.pose.position.y,drone_pose.pose.pose.position.z)
            print(P0)
            Q0 = Quaternion(drone_pose.pose.pose.orientation.x,drone_pose.pose.pose.orientation.y,drone_pose.pose.pose.orientation.z,drone_pose.pose.pose.orientation.w)

            V = Twist(linear=Vector3( 0, 0, 0), angular=Vector3( 0, 0, 0))
            A = Twist(linear=Vector3( 0, 0, 0), angular=Vector3( 0, 0, 0))
            T0 = Transform(translation=P0, rotation=Q0)
            trajP0 = MultiDOFJointTrajectoryPoint(transforms=[T0], velocities=[V], accelerations=[A])
            traj.points.append(trajP0)
            if count == 0 :
                print("Pubslished new")
                # pub1.publish(pubMsg1)
                pub2.publish(traj)
            count = count+1
            if count>32:
                pubMsg4 = tag_annotated
                pub4.publish(pubMsg4)
                pubMsg3.x = marker_pose.markers[0].pose.pose.position.x
                pubMsg3.y = marker_pose.markers[0].pose.pose.position.y
                pubMsg3.z = marker_pose.markers[0].pose.pose.position.z
                pub3.publish(pubMsg3)
                print("Final Position")
                print(pubMsg3.x,pubMsg3.y,pubMsg3.z)
                os.system("rosnode kill /zone3exp")
                rospy.sleep(5)
                
            
def zone3exp():
    rospy.init_node('zone3exp')
    global drone_pose,marker_pose, zone, tag_detected, count, detected, reached_last_point
    reached_last_point = False
    count = 0
    # TODO: change zone to -1
    zone = -1
    tag_detected = False
    drone_pose = None
    marker_pose= None
    waypoints_generation()
    rospy.Subscriber('/red/odometry',Odometry,odomCallback)
    rospy.Subscriber('/zone',Int32,zoneCallback)
    rospy.Subscriber('/ar_pose_marker',AlvarMarkers,markerCallback)
    rospy.Subscriber('/remap_tag_image_annotated',Image,annotatedCallback)
    flag = False
    # rate =rospy.Rate(10)
    while not rospy.is_shutdown():
        if drone_pose is not None and zone == 3:
            rospy.Timer(rospy.Duration(0.1),reachPointCallback,oneshot=True)
        if flag == False and zone == 3:
            exploration_trajectory()
            flag = True
        if reached_last_point == True and detected == False and zone == 3:
            rospy.Timer(rospy.Duration(3),regenerate_callback,oneshot = True)
            print("DIDNT DETECT HERE WE GO AGAIN")
            exploration_trajectory()
            reached_last_point = False
        if drone_pose is not None and zone == 3:
            detection()
            drone_pose = None
        # rate.sleep()


if __name__ == '__main__':
    try:
        zone3exp()
    except rospy.ROSInterruptException:
        pass