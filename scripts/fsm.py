#!/usr/bin/python

import rospy
import os
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Int32,Bool

fuel_worked=False
fuel_killed=False
fuel_last_trigger=-1
challenge_started=False
exp3_started=False
tag_detected=False
crr_pos=None
last_sampled_pose=None
last_sampled_time=0

def challenge_started_callback(data):
    global challenge_started
    challenge_started=data.data

def kill_node(node):
    os.system("rosnode kill "+node)

def triggerFUEL():
    os.system("gnome-terminal -- roslaunch exploration_manager icuas.launch init_x:="+str(crr_pos.x)+" init_y:="+str(crr_pos.y)+" init_z:="+str(crr_pos.z))
    os.system("gnome-terminal -- rosrun icuas22_competition trigger_FUEL.py")

def poseCallback(data):
    global crr_pos, last_sampled_pose, last_sampled_time
    crr_pos = data.pose.pose.position
    if last_sampled_pose is None or rospy.get_time()-last_sampled_time>10:
        last_sampled_pose=crr_pos
        last_sampled_time=rospy.get_time()

def dist(x, y):
    return np.sqrt((x.x-y.x)**2+(x.y-y.y)**2+(x.z-y.z)**2)

def zoneCallback(data):
    global fuel_worked, zone
    zone = data.data
    if zone != 1:
        fuel_worked=True

def tagCallback(data):
    global tag_detected
    tag_detected = data.data

def doStuff():
    global fuel_last_trigger, fuel_killed, exp3_started, last_moving_time, tag_detected
    while crr_pos is None:
        return
    if zone==3:
        if not fuel_killed:
            kill_node("waypoint_generator")
            kill_node("traj_server")
            kill_node("exploration_node")
            kill_node("trigger_FUEL")
            kill_node("sensor_pose")
            kill_node("pos_to_multiDOF")
            fuel_killed=True
        if not exp3_started:
            os.system("gnome-terminal -- rosrun icuas22_competition zone3exp.py")
            exp3_started=True
        if tag_detected:
            print("tag detected")
            kill_node("isDetected")
            kill_node("fsm")
    elif fuel_last_trigger==-1:
        print("triggering fuel")
        fuel_last_trigger=rospy.get_time()
        triggerFUEL()
    elif dist(crr_pos,last_sampled_pose)<0.1 and rospy.get_time()-last_sampled_time>5 and rospy.get_time()-fuel_last_trigger>5:
        print("triggering fuel")
        kill_node("waypoint_generator")
        kill_node("traj_server")
        kill_node("exploration_node")
        fuel_last_trigger=rospy.get_time()
        triggerFUEL()
    # if zone==1 and not fuel_worked:
    #     elif rospy.get_time()-fuel_last_trigger>10:
    #         print("triggering fuel")
    #         kill_node("waypoint_generator")
    #         kill_node("traj_server")
    #         kill_node("exploration_node")
    #         fuel_last_trigger=rospy.get_time()
    #         triggerFUEL()

def check():

    rospy.init_node("fsm")
    print("fsm node started")

    rospy.Subscriber("/zone", Int32, zoneCallback)
    rospy.Subscriber("/red/challenge_started", Bool, challenge_started_callback)
    rospy.Subscriber("/isDetected", Bool, tagCallback)
    rospy.Subscriber("/red/odometry", Odometry, poseCallback)
    while not rospy.is_shutdown():
        if challenge_started:
            doStuff()

if __name__ == "__main__":
    check()