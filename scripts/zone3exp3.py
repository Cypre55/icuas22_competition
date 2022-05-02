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
    [[3, -4, 3], [0, 0, -0.707, 0.707]],
    [[5, -4, 3], [0, 0, -0.707, 0.707]],
    [[7.5, -4, 3], [0, 0, -0.707, 0.707]],
    [[7, -4, 3], [0, 0, -0.707, 0.707]],
    [[9.5, -4, 3], [0, 0, -0.707, 0.707]],
    [[9, -4, 3], [0, 0, -0.707, 0.707]],
    [[9, -4, 3], [0, 0, -0.383, 0.924]],
    [[9.5, -4, 3], [0, 0, 0, 1]],
    [[9, -3.75, 3], [0, 0, 0, 1]],
    [[9, -2, 3], [0, 0, 0, 1]],
    [[9, 0, 3], [0, 0, 0, 1]],
    [[9, 2, 3], [0, 0, 0, 1]],
    [[9, 3.75, 3], [0, 0, 0, 1]],
    [[9.5, 4, 3], [0, 0, 0, 1]],
    [[9, 4, 3], [0, 0, 0.383, 0.924]],
    [[9, 4, 3], [0, 0, 0.707, 0.707]],
    [[9.5, 4, 3], [0, 0, 0.707, 0.707]],
    [[7, 4, 3], [0, 0, 0.707, 0.707]],
    [[7.5, 4, 3], [0, 0, 0.707, 0.707]],
    [[5, 4, 3], [0, 0, 0.707, 0.707]],
    [[2.5, 4, 3], [0, 0, 0.707, 0.707]]
]
REACH_THRESHOLD = 0.15

pub1 = rospy.Publisher('red/tracker/input_pose', PoseStamped, queue_size=10)
pub2 = rospy.Publisher('/red/tracker/input_trajectory',MultiDOFJointTrajectory,queue_size=10)
pub3 = rospy.Publisher('/red/tag_position_reconstructed',Point,queue_size=10)
detected = False
reached_last_point = False

def reach_point(x,y):
    global drone_pose,REACH_THRESHOLD
    if(abs(drone_pose.pose.pose.position.x - x) < REACH_THRESHOLD and abs(drone_pose.pose.pose.position.y - y) < REACH_THRESHOLD):
        return True
    else:
        return False


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
    global reached_last_point
    # if reached_last_point:
    exploration_trajectory()
    print("NEW TRAJECTORY CALLED")

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
    print("WAY PUB")
    pub2.publish(traj)

def detection():
    global marker_pose,drone_pose, count,detected
    pubMsg3 = Point()
    traj = MultiDOFJointTrajectory()
    
    if marker_pose is not None:
        if marker_pose.markers:
            print("Detected")
            detected = True
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
            if count == 0:
                print("Pubslished new")
                pub2.publish(traj)
            count = count+1
            if count>10:
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
    zone = -1
    tag_detected = False
    drone_pose = None
    marker_pose= None
    rospy.Subscriber('/red/odometry',Odometry,odomCallback)
    rospy.Subscriber('/zone',Int32,zoneCallback)
    rospy.Subscriber('/ar_pose_marker',AlvarMarkers,markerCallback)
    flag = False
    rate =rospy.Rate(10)
    while not rospy.is_shutdown():
        # print("zone: ",zone)
        if flag == False and zone == 3:
            exploration_trajectory()
            flag = True
            i=1
        if reached_last_point == True and detected == False and zone == 3:
            rospy.Timer(rospy.Duration(5),regenerate_callback,oneshot = True)
            print("DIDNT DETECT HERE WE GO AGAIN")
            reached_last_point = False
            
        if drone_pose is not None and zone == 3:
            detection()
            reached_last_point = reach_point(WAYPOINTS[8][0][0],WAYPOINTS[8][0][1])
            drone_pose = None
        rate.sleep()


if __name__ == '__main__':
    try:
        zone3exp()
    except rospy.ROSInterruptException:
        pass