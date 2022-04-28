#!/usr/bin/env python

__author__ = 'ishabhray'

import rospy
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':

    rospy.init_node('trigger_FUEL')
    trigger = PoseStamped()
    triggerPub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    while not rospy.is_shutdown():
        triggerPub.publish(trigger)
        rospy.sleep(.5)