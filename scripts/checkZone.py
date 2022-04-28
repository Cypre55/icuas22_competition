#!/usr/bin/python

import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Int32,Bool

zone=-1
started=False
def checkCallback(data):
    global zone,started
    x=data.pose.pose.position.x
    zone=-1
    if started:
        if x > -8 and x < 1:
            zone=2
        elif x > 1 and x < 12.5:
            zone=3
        else:
            zone=1

def startCallback(data):
    global started
    if data.data:
        started = True

def check():
    global zone

    rospy.init_node("check_zone")
    rospy.Subscriber("/red/challenge_started", Bool, startCallback)
    rospy.Subscriber("/red/odometry", Odometry, checkCallback)
    zonePub = rospy.Publisher("/zone", Int32, queue_size=3)
    while not rospy.is_shutdown():
        zonePub.publish(zone)

if __name__ == "__main__":
    check()