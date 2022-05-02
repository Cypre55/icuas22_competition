#!/usr/bin/env python

from matplotlib import transforms
import rospy
from math import cos, sin, sqrt, pi
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty, SetBool
from std_msgs.msg import Float32, Duration, Bool, String
from geometry_msgs.msg import PoseStamped, Point, Vector3, Twist, Quaternion, Transform
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint, MultiDOFJointTrajectory
from tf.transformations import euler_from_quaternion, quaternion_from_euler

signalToDrop = 0
ball_to_uav_offset = 0.4 #
minDist = 100
ballistic = False
z = -1000
crr_pos = Vector3()
tile_yaw = 0
tile_x = 0
tile_y = 0
tile_z = 0
yaw_flag = 0
yaw_ = 0
detected = False
status_ = ""


def arDetected(msg):
    pass

def tagCallback(msg):
    global tile_x, tile_y, tile_z, tile_yaw, yaw_flag, detected
    tile_x = msg.x
    tile_y = msg.y
    tile_z = msg.z

    if yaw_flag == 0:
        yaw_flag = 1
        if abs(tile_x - 12.5) <= 0.15:
            tile_yaw = 0
        elif abs(tile_y - 7.5) <= 0.15:
            tile_yaw = 1.57079632679
        elif abs(tile_y + 7.5) <= 0.15:
            tile_yaw = -1.57079632679
        else:
            print("unable to set yaw")
        print("tile_x = ", tile_x)
        print("tile_y = ", tile_y)
        print("tile_z = ", tile_z)
        print("tile_yaw = ", tile_yaw)

    detected = True

def odometryCallback(msg):
    global signalToDrop, crr_pos, tile_z, minDist, ballistic, z, yaw_

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z

    Qx = msg.pose.pose.orientation.x
    Qy = msg.pose.pose.orientation.y
    Qz = msg.pose.pose.orientation.z
    Qw = msg.pose.pose.orientation.w

    yaw_ = euler_from_quaternion([Qx, Qy, Qz, Qw])[2]

    velx = msg.twist.twist.linear.x
    vely = msg.twist.twist.linear.y
    velz = msg.twist.twist.linear.z

    angx = msg.twist.twist.angular.x
    angy = msg.twist.twist.angular.y
    angz = msg.twist.twist.angular.z

    crr_pos = msg.pose.pose.position
    
    err = 0.075
    tune_fix = 0.00

    if (z+tune_fix) - (tile_z+ball_to_uav_offset)>= err and ballistic:
        signalToDrop = 1

def ballOdom(msg):
    global tile_x, tile_y, tile_z, minDist

    xb = msg.pose.position.x
    yb = msg.pose.position.y
    zb = msg.pose.position.z

    Dist = sqrt( ( xb - (tile_x) )**2 + ( yb  - (tile_y))**2 + (tile_z - zb)**2 ) - 0.1

    if Dist < minDist:
        minDist = Dist
        print("minDist = ", minDist)
    # if Dist > minDist:
    #     with open("test_results.txt","a")as f:
    #         f.write(str(minDist))


def generateTraj1():
    global tile_x, tile_y, tile_z, tile_yaw
    global crr_pos

    traj = MultiDOFJointTrajectory()

    P1 = Vector3( tile_x - 10*cos(tile_yaw), tile_y - 10*sin(tile_yaw), tile_z + ball_to_uav_offset - 0.3)
    Q = quaternion_from_euler(0.0, 0.0, tile_yaw)
    Q1 = Quaternion(Q[0], Q[1], Q[2], Q[3])
    T1 = Transform(translation=P1, rotation=Q1)
    V1 = Twist(linear=Vector3( 0, 0, 0), angular=Vector3( 0, 0, 0))
    A1 = Twist(linear=Vector3( 0, 0, 0), angular=Vector3( 0, 0, 0))

    T_ = Transform(translation=crr_pos, rotation=Q1)

    trajP0 = MultiDOFJointTrajectoryPoint(transforms=[T_], velocities=[V1], accelerations=[A1])
    trajP1 = MultiDOFJointTrajectoryPoint(transforms=[T1], velocities=[V1], accelerations=[A1])
    
    traj.points.append(trajP0)
    traj.points.append(trajP1)

    return traj


def generateTraj():
    global tile_x, tile_y, tile_z, tile_yaw
    global crr_pos

    P1 = Vector3( tile_x - 10*cos(tile_yaw), tile_y - 10*sin(tile_yaw), tile_z + ball_to_uav_offset - 0.3)
    P2 = Vector3( tile_x + 0.75*cos(tile_yaw), tile_y + 0.75*sin(tile_yaw) , tile_z + ball_to_uav_offset + 1.2)
    P3 = Vector3( tile_x - 5*cos(tile_yaw), tile_y - 5*sin(tile_yaw), tile_z + ball_to_uav_offset + 0.6)

    Q = quaternion_from_euler(0.0, 0.0, tile_yaw)
    Q1 = Quaternion(Q[0], Q[1], Q[2], Q[3])

    V1 = Twist(linear=Vector3( 0, 0, 0), angular=Vector3( 0, 0, 0))
    V2 = Twist(linear=Vector3( 15*cos(tile_yaw), 15*sin(tile_yaw), 100), angular=Vector3( 0, 0, 0))
    V3 = Twist(linear=Vector3( 0, 0, 0), angular=Vector3( 0, 0, 0))

    A1 = Twist(linear=Vector3( 20*cos(tile_yaw), 20*sin(tile_yaw), 0), angular=Vector3( 0, 0, 0))
    A2 = Twist(linear=Vector3( 0*cos(tile_yaw), 0*sin(tile_yaw), 100), angular=Vector3( 0, 0, 0))
    A3 = Twist(linear=Vector3( 0, 0, 0), angular=Vector3( 0, 0, 0))

    T1 = Transform(translation=P1, rotation=Q1)
    T2 = Transform(translation=P2, rotation=Q1)
    T3 = Transform(translation=P3, rotation=Q1)
    
    traj = MultiDOFJointTrajectory()

    trajP1 = MultiDOFJointTrajectoryPoint(transforms=[T1], velocities=[V1], accelerations=[A1])
    trajP2 = MultiDOFJointTrajectoryPoint(transforms=[T2], velocities=[V2], accelerations=[A2])
    trajP3 = MultiDOFJointTrajectoryPoint(transforms=[T3], velocities=[V3], accelerations=[A3])

    traj.points.append(trajP1)
    traj.points.append(trajP2)
    traj.points.append(trajP3)

    return traj

def statusCallback(msg):
    global status_
    status_ = msg.data

def clostTo(X, Y):
    if (sqrt( (X.x - Y.x)**2 + (X.y - Y.y)**2 + (X.z - Y.z)**2 ) <= 0.4):
        return True
    else:
        return False

def landNow():
    
    land = rospy.ServiceProxy('/red/land',SetBool)
    rospy.sleep(1)
    land.call(True)


def main():
    
    global signalToDrop, ballistic, crr_pos, inAir

    rospy.init_node("precisionDelivery_node")

    rospy.Subscriber("/red/ball/pose", PoseStamped, ballOdom)
    rospy.Subscriber("/red/odometry", Odometry, odometryCallback)
    rospy.Subscriber("/isDetected", Bool, arDetected)
    rospy.Subscriber("/red/tag_position_reconstructed", Point, tagCallback)
    rospy.Subscriber("/red/carrot/status", String, statusCallback)    

    trajPub = rospy.Publisher("/red/position_hold/trajectory", MultiDOFJointTrajectoryPoint, queue_size=1)
    trajPub1 = rospy.Publisher("/red/tracker/input_trajectory", MultiDOFJointTrajectory, queue_size=1)
    magnetPub = rospy.Publisher("/red/uav_magnet/gain", Float32, queue_size=1)
    distPub = rospy.Publisher("/red/ball_min_distance", Float32, queue_size=1)
    rate = rospy.Rate(50) #Hz

    while not rospy.is_shutdown() and not detected:
        continue

    if detected:
        
        rospy.loginfo("Starting Ball Delivery.")

        print("tile_x = ", tile_x)
        print("tile_y = ", tile_y)
        print("tile_z = ", tile_z)
        print("tile_yaw = ", tile_yaw)

        traj = generateTraj1()
        traj1 = generateTraj()
    
        #trajPub1.publish(traj)
        
        while not clostTo(traj.points[1].transforms[0].translation, crr_pos) and not rospy.is_shutdown():
            #print("not at position")
            trajPub.publish(traj.points[1])
            rate.sleep()

        rospy.loginfo("Initiating Ballistic Trajectory.")
        
        trajPub1.publish(traj1)
        ballistic = True
        
        inAir = True
        landed=False
        while not rospy.is_shutdown():
            distPub.publish(minDist)
            if signalToDrop == 1:
                magnetPub.publish(0.0)
                if inAir == True:
                    rospy.loginfo("Landing Initiated.")
                    distPub.publish(minDist)
                    inAir = False
                    if not landed:
                        landNow()
                        landed = True
            rate.sleep()        

if __name__ == '__main__':
    main()