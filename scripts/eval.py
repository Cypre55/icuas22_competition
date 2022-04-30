#!/usr/bin/env python

from math import exp, sqrt
import rospy
import os, sys
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Point

startTime = None
endTime = None
minDist = 100
tagDist = 100

def startCallback(data):
    global startTime
    if data.data and startTime is None:
        startTime = rospy.Time.now()

def endCallback(data):
    global endTime
    if data.data == 0. and endTime is None:
        endTime = rospy.Time.now()

def distCallback(data):
    global minDist
    minDist = data.data

def tagCallback(data):
    global tagDist
    tile_x = data.x
    tile_y = data.y
    tile_z = data.z
    x=float(sys.argv[1])
    y=float(sys.argv[2])
    z=float(sys.argv[3])
    x-=tile_x
    y-=tile_y
    z-=tile_z
    tagDist = float(sqrt(x**2+y**2+z**2))

def calc(time, hit, tag):
    score = 0
    score += round(25*exp(-2*tag)) * 2 / 2
    score += round(35*exp(-0.5*hit)) * 2 / 2
    score += round(40*exp(-time/100)) * 2 / 2
    return score

def eval():
    rospy.init_node('eval', anonymous=True)
    rospy.Subscriber('/red/challenge_started',Bool,startCallback)
    rospy.Subscriber('/red/uav_magnet/gain',Float32,endCallback)
    rospy.Subscriber('/red/ball_min_distance',Float32,distCallback)
    rospy.Subscriber('/red/tag_position_reconstructed',Point,tagCallback)

    while not rospy.is_shutdown():
        if startTime is not None and endTime is not None:
            rospy.loginfo("Time taken: %f", (endTime-startTime).to_sec())
            rospy.loginfo("Hit distance: %f", minDist)
            rospy.loginfo("Tag distance: %f", tagDist)
            rospy.loginfo("Score: %f", calc((endTime-startTime).to_sec(), minDist, tagDist))
            break
        rospy.sleep(0.1)

if __name__ == '__main__':
    eval()