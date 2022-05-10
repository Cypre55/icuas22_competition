#!/usr/bin/env python
from itertools import count
from re import S
from tracemalloc import start
import rospy,os
from geometry_msgs.msg import PoseStamped, Point, Vector3, Quaternion, Twist, Transform
from tf.transformations import euler_from_quaternion, quaternion_from_euler 
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from nav_msgs.msg import Odometry
from ar_track_alvar_msgs.msg import AlvarMarkers,AlvarMarker
from std_msgs.msg import Int32, Bool
import numpy as np
from sensor_msgs.msg import Image

zone = -1
start = False
WAYPOINTS = []

def waypoints_generation():
    z = 2
    # left to right
    iterations = 10
    for i in range(10):
        WAYPOINTS.append([[-9, -5, z], [0, 0, 0.5, 0.866]])
    for i in range(iterations):
        if i%2==0:
            WAYPOINTS.append([[-11, -5+(10)*i/(iterations-1), z], [0, 0, 0, 1]])
        else:
            WAYPOINTS.append([[-9, -5+(10)*i/(iterations-1), z+0.5], [0, 0, 0, 1]])
    # right to left
    for i in range(10):
        WAYPOINTS.append([[-11, 5, z], [0, 0, 0, 1]])
    iterations = 10
    for i in range(iterations):
        if i%2==0:
            WAYPOINTS.append([[-11, 5+(-10)*i/(iterations-1), z], [0, 0, 0, 1]])
        else:
            WAYPOINTS.append([[-9, 5+(-10)*i/(iterations-1), z+0.5], [0, 0, 0, 1]])
    for i in range(10):
        WAYPOINTS.append([[-9, -5, z], [0, 0, 0.5, 0.866]])
    for i in range(10):
        WAYPOINTS.append([[-9, 5, z], [0, 0, -0.5, 0.866]])

REACH_THRESHOLD = 0.4

pub1 = rospy.Publisher('red/tracker/input_pose', PoseStamped, queue_size=10)
pub2 = rospy.Publisher('/red/tracker/input_trajectory',MultiDOFJointTrajectory,queue_size=10)
reached_last_point = False

def reach_point(x,y):
    global drone_pose,REACH_THRESHOLD
    if(abs(drone_pose.pose.pose.position.x - x) < REACH_THRESHOLD and abs(drone_pose.pose.pose.position.y - y) < REACH_THRESHOLD):
        return True
    else:
        return False

def odomCallback(data):
    global drone_pose
    drone_pose = data

def zoneCallback(data):
    global zone
    zone = data.data

def challengeCallback(data):
    global start
    start = data.data
    if start:
        rospy.loginfo("Challenge Started")

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

def zone1exp():
    rospy.init_node('zone1exp')
    global drone_pose,zone,reached_last_point,start
    reached_last_point = False
    # TODO: change zone to -1
    drone_pose = None
    waypoints_generation()
    rospy.Subscriber('/red/odometry',Odometry,odomCallback)
    rospy.Subscriber('/zone',Int32,zoneCallback)
    rospy.Subscriber('/red/challenge_started',Bool,challengeCallback)
    flag = False
    # rate =rospy.Rate(10)
    while not rospy.is_shutdown():
        if not start:
            continue
        if flag == False and zone == 1:
            print("HERE WE GO@!!!!!")
            exploration_trajectory()
            flag = True
        # if reached_last_point == True and detected == False and zone == 3:
        #     rospy.Timer(rospy.Duration(3),regenerate_callback,oneshot = True)
        #     print("DIDNT DETECT HERE WE GO AGAIN")
        #     exploration_trajectory()
        #     reached_last_point = False
        # if drone_pose is not None and zone == 3:
        #     drone_pose = None
        # rate.sleep()


if __name__ == '__main__':
    try:
        zone1exp()
    except rospy.ROSInterruptException:
        pass