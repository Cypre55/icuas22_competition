#!/usr/bin/env python

from matplotlib import transforms
import rospy
from time import sleep
from math import cos, sin, sqrt
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool, Empty
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, Point, Vector3, Twist, Quaternion, Transform, Pose
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
b =0.2

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
    
    err = 0.0
    tune_fix = 0.00

    # and z >= tile_z + ball_to_uav_offset - 0.4
    a = 2.27
    b = 0.15
    if x*cos(tile_yaw) + y*sin(tile_yaw) > ((tile_x)*cos(tile_yaw) - a)*abs(cos(tile_yaw)) + ((tile_y)*sin(tile_yaw) - a)*abs(sin(tile_yaw)) and abs(z - (tile_z + ball_to_uav_offset)) >= b  and ballistic:
        signalToDrop = 1



def ballOdom(msg):
    global tile_x, tile_y, tile_z, minDist

    xb = msg.pose.position.x
    yb = msg.pose.position.y
    zb = msg.pose.position.z
    # Dist = sqrt( ( xb - (tile_x) )**2 + ( yb  - (tile_y))**2 + (tile_z - zb)**2 )
    Dist = sqrt( ( xb + 0.1*cos(tile_yaw) - (tile_x) )**2 + ( yb + 0.1*sin(tile_yaw)  - (tile_y))**2 + (tile_z - zb)**2 )


    if Dist < minDist:
        minDist = Dist
        print("minDist = ", minDist)
    # if Dist > minDist:
    #     with open("test_results.txt","a")as f:
    #         f.write(str(minDist))


def waypointFromWall(dist, z):
    return Vector3((tile_x) - dist*cos(tile_yaw)+ 0.1*sin(tile_yaw), (tile_y) - dist*sin(tile_yaw) + 0.1*cos(tile_yaw), z)

def generateTraj():
    global tile_x, tile_y, tile_z, tile_yaw
    global crr_pos

    traj = MultiDOFJointTrajectory()

    #range R = 1m, vel = 3m/s

    Q = quaternion_from_euler(0.0, 0.0, tile_yaw)
    Q1 = Quaternion(Q[0], Q[1], Q[2], Q[3])

    P1 = waypointFromWall( 8,               tile_z  + ball_to_uav_offset - 1 + (0)  )   #back P1
    P2 = waypointFromWall( 4,               tile_z  + ball_to_uav_offset - 1 + 0.5    )   #back P2
    P3 = waypointFromWall( 1.3,          tile_z  + ball_to_uav_offset      + 0 + 0.8 )   #Continue straight from here
    # P6 = waypointFromWall( 2,          tile_z  + ball_to_uav_offset      + 0 + 0.6 )   #Continue straight from here
    P4 = waypointFromWall( 3,             tile_z  + ball_to_uav_offset + 0.5 )   #end straight path
    P5 = waypointFromWall( 10,               tile_z  + ball_to_uav_offset + 0.5 )   #stopping point

    T1 = Transform(translation=P1, rotation=Q1)
    T2 = Transform(translation=P2, rotation=Q1)
    T3 = Transform(translation=P3, rotation=Q1)
    # T6 = Transform(translation=P6, rotation=Q1)
    T4 = Transform(translation=P4, rotation=Q1)
    T5 = Transform(translation=P5, rotation=Q1)

    V1 = Twist(linear=Vector3( 0, 0, 0),                                               angular=Vector3( 0, 0, 0))
    V2 = Twist(linear=Vector3( 5*round(cos(tile_yaw)),         5*round(sin(tile_yaw)),        0),     angular=Vector3( 0, 0, 0))
    V3 = Twist(linear=Vector3( 5*round(cos(tile_yaw)),         5*round(sin(tile_yaw)),        4.2),     angular=Vector3( 0, 0, 0))
    # V6 = Twist(linear=Vector3( -2*cos(tile_yaw),   -2*sin(tile_yaw),  0),     angular=Vector3( 0, 0, 0))
    V4 = Twist(linear=Vector3( -6*round(cos(tile_yaw)),   -6*round(sin(tile_yaw)),  0),     angular=Vector3( 0, 0, 0))
    V5 = Twist(linear=Vector3( 0, 0, 0),                                                angular=Vector3( 0, 0, 0))

    A1 = Twist(linear=Vector3( 0, 0, 0), angular=Vector3( 0, 0, 0))
    A2 = Twist(linear=Vector3( 0, 0, 0), angular=Vector3( 0, 0, 0))
    A3 = Twist(linear=Vector3( -1*round(cos(tile_yaw)), -1*round(sin(tile_yaw)), 0), angular=Vector3( 0, 0, 0))
    # A6 = Twist(linear=Vector3( -1.5*cos(tile_yaw), -1.5*sin(tile_yaw), 0), angular=Vector3( 0, 0, 0))
    A4 = Twist(linear=Vector3( 0, 0, 0), angular=Vector3( 0, 0, 0))
    A5 = Twist(linear=Vector3( 0, 0, 0), angular=Vector3( 0, 0, 0))

    trajP1 = MultiDOFJointTrajectoryPoint(transforms=[T1], velocities=[V1], accelerations=[A1])
    trajP2 = MultiDOFJointTrajectoryPoint(transforms=[T2], velocities=[V2], accelerations=[A2])
    trajP3 = MultiDOFJointTrajectoryPoint(transforms=[T3], velocities=[V3], accelerations=[A3])
    # trajP6 = MultiDOFJointTrajectoryPoint(transforms=[T6], velocities=[V6], accelerations=[A6])
    trajP4 = MultiDOFJointTrajectoryPoint(transforms=[T4], velocities=[V4], accelerations=[A4])
    trajP5 = MultiDOFJointTrajectoryPoint(transforms=[T5], velocities=[V5], accelerations=[A5])

    traj.points.append(trajP1)
    traj.points.append(trajP2)
    traj.points.append(trajP3)
    # traj.points.append(trajP6)
    traj.points.append(trajP4)
    traj.points.append(trajP5)

    return traj


def clostTo(X, Y):
    if abs(X.x - Y.x) <= 0.05 and abs(X.y - Y.y) <= 0.05:
        return True
    else:
        return False

def landNow():
    
    land = rospy.ServiceProxy('/red/land',SetBool)
    rospy.sleep(1)
    land.call(True)


def main():
    
    global signalToDrop, ballistic, crr_pos, inAir

    rospy.init_node("delivery_node")

    rospy.Subscriber("/red/ball/pose", PoseStamped, ballOdom)
    rospy.Subscriber("/red/odometry", Odometry, odometryCallback)
    rospy.Subscriber("/red/tag_position_reconstructed", Point, tagCallback)    

    # trajPub = rospy.Publisher("/red/position_hold/trajectory", MultiDOFJointTrajectoryPoint, queue_size=1)
    posePub = rospy.Publisher("/red/tracker/input_trajectory", MultiDOFJointTrajectory, queue_size=1)
    trajPub1 = rospy.Publisher("/red/tracker/input_trajectory2", MultiDOFJointTrajectory, queue_size=1)
    magnetPub = rospy.Publisher("/red/uav_magnet/gain", Float32, queue_size=1)
    distPub = rospy.Publisher("/red/ball_min_distance", Float32, queue_size=30)

    trajReset = rospy.ServiceProxy('/red/tracker/reset', Empty)

    rate = rospy.Rate(1000) #Hz

    rospy.sleep(1) #comment this out or not for the final integration with FSM

    while not rospy.is_shutdown() and not detected:
        continue

    if detected:
        
        rospy.loginfo("Starting Ball Delivery.")

        srvResp = trajReset()
        # if srvResp == 

        print("tile_x = ", tile_x)
        print("tile_y = ", tile_y)
        print("tile_z = ", tile_z)
        print("tile_yaw = ", tile_yaw)

        traj = generateTraj()

        P0 = waypointFromWall( 8,               tile_z  + ball_to_uav_offset - 1 + (0))
        Q = quaternion_from_euler(0,0,tile_yaw)
        Q0 = Quaternion(Q[0], Q[1], Q[2], Q[3])
        T0 = Transform(translation=P0, rotation=Q0)
        pose0 = MultiDOFJointTrajectoryPoint(transforms=[T0])
        Pose0 = MultiDOFJointTrajectory()
        Pose0.points.append(pose0)

        posePub.publish(Pose0)
    
        for i in range(7):
            rospy.sleep(1)
            print("sleep")

        trajPub1.publish(traj)
        
        # while not clostTo(traj.points[1].transforms[0].translation, crr_pos) and not rospy.is_shutdown():
        #     #print("not at position")
        #     trajPub.publish(traj.points[1])
        #     rate.sleep()

        rospy.loginfo("Initiating Ballistic Trajectory.")
        
        ballistic = True
        
        inAir=True
        landed=False

        while not rospy.is_shutdown():
            
            if signalToDrop == 1:
                #print("yes")
                magnetPub.publish(0.0)

                if inAir == True:
                    rospy.loginfo("Landing Initiated.")
                    inAir = False

                    if not landed:
                        rospy.sleep(4)
                        #landNow()
                        distPub.publish(minDist)
                        landed = True
                        
            rate.sleep()        

if __name__ == '__main__':
    main()