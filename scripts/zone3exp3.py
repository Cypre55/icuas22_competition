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

zone = -1
WAYPOINTS = [
    [[2, -4, 3], [0, 0, -0.682, 0.732]],
    [[9, -4, 3], [0, 0, -0.682, 0.732]],
    [[9, -4, 3], [0, 0, 0, 1]],
    [[9, 4, 3], [0, 0, 0, 1]],
    [[9, 4, 3], [0, 0, 0.682, 0.732]],
    [[2, 4, 3], [0, 0, 0.682, 0.732]]
]
REACH_THRESHOLD = 0.2

pub1 = rospy.Publisher('red/tracker/input_pose', PoseStamped, queue_size=10)
pub2 = rospy.Publisher('/red/tracker/input_trajectory',MultiDOFJointTrajectory,queue_size=10)
pub3 = rospy.Publisher('/red/tag_position_reconstructed',Point,queue_size=10)

def markerCallback(data):
    global marker_pose
    marker_pose = data

def odomCallback(data):
    global drone_pose
    drone_pose = data

def zoneCallback(data):
    global zone
    zone = data.data

def exploration_trajectory():
    traj = MultiDOFJointTrajectory()

    P0 = Vector3(WAYPOINTS[0][0][0],WAYPOINTS[0][0][1],WAYPOINTS[0][0][2])
    P1 = Vector3(WAYPOINTS[1][0][0],WAYPOINTS[1][0][1],WAYPOINTS[1][0][2])
    P2 = Vector3(WAYPOINTS[2][0][0],WAYPOINTS[2][0][1],WAYPOINTS[2][0][2])
    P3 = Vector3(WAYPOINTS[3][0][0],WAYPOINTS[3][0][1],WAYPOINTS[3][0][2])
    P4 = Vector3(WAYPOINTS[4][0][0],WAYPOINTS[4][0][1],WAYPOINTS[4][0][2])
    P5 = Vector3(WAYPOINTS[5][0][0],WAYPOINTS[5][0][1],WAYPOINTS[5][0][2])

    Q0 = Quaternion(WAYPOINTS[0][1][0],WAYPOINTS[0][1][1],WAYPOINTS[0][1][2],WAYPOINTS[0][1][3])
    Q1 = Quaternion(WAYPOINTS[1][1][0],WAYPOINTS[1][1][1],WAYPOINTS[1][1][2],WAYPOINTS[1][1][3])
    Q2 = Quaternion(WAYPOINTS[2][1][0],WAYPOINTS[2][1][1],WAYPOINTS[2][1][2],WAYPOINTS[2][1][3])
    Q3 = Quaternion(WAYPOINTS[3][1][0],WAYPOINTS[3][1][1],WAYPOINTS[3][1][2],WAYPOINTS[3][1][3])
    Q4 = Quaternion(WAYPOINTS[4][1][0],WAYPOINTS[4][1][1],WAYPOINTS[4][1][2],WAYPOINTS[4][1][3])
    Q5 = Quaternion(WAYPOINTS[5][1][0],WAYPOINTS[5][1][1],WAYPOINTS[5][1][2],WAYPOINTS[5][1][3])

    V = Twist(linear=Vector3( 0, 0, 0), angular=Vector3( 0, 0, 0))
    A = Twist(linear=Vector3( 0, 0, 0), angular=Vector3( 0, 0, 0))
    T0 = Transform(translation=P0, rotation=Q0)
    T1 = Transform(translation=P1, rotation=Q1)
    T2 = Transform(translation=P2, rotation=Q2)
    T3 = Transform(translation=P3, rotation=Q3)
    T4 = Transform(translation=P4, rotation=Q4)
    T5 = Transform(translation=P5, rotation=Q5)

    trajP0 = MultiDOFJointTrajectoryPoint(transforms=[T0], velocities=[V], accelerations=[A])
    trajP1 = MultiDOFJointTrajectoryPoint(transforms=[T1], velocities=[V], accelerations=[A])
    trajP2 = MultiDOFJointTrajectoryPoint(transforms=[T2], velocities=[V], accelerations=[A])
    trajP3 = MultiDOFJointTrajectoryPoint(transforms=[T3], velocities=[V], accelerations=[A])
    trajP4 = MultiDOFJointTrajectoryPoint(transforms=[T4], velocities=[V], accelerations=[A])
    trajP5 = MultiDOFJointTrajectoryPoint(transforms=[T5], velocities=[V], accelerations=[A])
    traj.points.append(trajP0)
    traj.points.append(trajP1)
    traj.points.append(trajP2)
    traj.points.append(trajP3)
    traj.points.append(trajP4)
    traj.points.append(trajP5)
    pub2.publish(traj)

def detection():
    global marker_pose,drone_pose, count
    pubMsg3 = Point()
    traj = MultiDOFJointTrajectory()
    if marker_pose is not None:
        if marker_pose.markers:
            print("Detected")
            a = marker_pose.markers[0].pose.pose.position.x
            b = marker_pose.markers[0].pose.pose.position.y
            c = marker_pose.markers[0].pose.pose.position.z
            print(a,b,c)
            P0 = Vector3(drone_pose.pose.pose.position.x,drone_pose.pose.pose.position.y,drone_pose.pose.pose.position.z)
            print(P0)
            Q0 = Quaternion(drone_pose.pose.pose.orientation.x,drone_pose.pose.pose.orientation.y,drone_pose.pose.pose.orientation.z,drone_pose.pose.pose.orientation.w)

            V = Twist(linear=Vector3( 0, 0, 0), angular=Vector3( 0, 0, 0))
            A = Twist(linear=Vector3( 0, 0, 0), angular=Vector3( 0, 0, 0))
            T0 = Transform(translation=P0, rotation=Q0)
            trajP0 = MultiDOFJointTrajectoryPoint(transforms=[T0], velocities=[V], accelerations=[A])
            traj.points.append(trajP0)
            if count < 5:
                pub2.publish(traj)
            count = count+1
            if count>20:
                pubMsg3.x = marker_pose.markers[0].pose.pose.position.x
                pubMsg3.y = marker_pose.markers[0].pose.pose.position.y
                pubMsg3.z = marker_pose.markers[0].pose.pose.position.z
                pub3.publish(pubMsg3)
                print("Final Position")
                print(pubMsg3.x,pubMsg3.y,pubMsg3.z)
                os.system("rosnode kill /zone3exp")
                
            
def zone3exp():
    rospy.init_node('zone3exp')
    global drone_pose,marker_pose, zone, tag_detected, count
    count = 0
    zone = -1
    tag_detected = False
    drone_pose = None
    marker_pose= None
    rospy.Subscriber('/red/odometry',Odometry,odomCallback)
    rospy.Subscriber('/zone',Int32,zoneCallback)
    rospy.Subscriber('/ar_pose_marker',AlvarMarkers,markerCallback)
    flag = False
    rate =rospy.Rate(10)
    while not rospy.is_shutdown() :
        print("zone: ",zone)
        if flag == False and zone ==3:
            exploration_trajectory()
            flag = True
        if drone_pose is not None and zone ==3:
            detection()
            drone_pose = None
        rate.sleep()


if __name__ == '__main__':
    try:
        zone3exp()
    except rospy.ROSInterruptException:
        pass