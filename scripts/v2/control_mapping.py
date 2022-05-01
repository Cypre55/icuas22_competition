#!/usr/bin/python

import rospy

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Transform

from std_msgs.msg import Int32, Bool

import tf.transformations

from quadrotor_msgs.msg import PositionCommand
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint

def zoneCallback(data):
    global zone
    zone = data.data

def convertCallback(data):
    rot = tf.transformations.quaternion_from_euler(0, 0, data.yaw)

    multiDOF = MultiDOFJointTrajectoryPoint()

    pos = Vector3()
    vel = Vector3()
    ang = Vector3()
    acc = Vector3()

    _vel = Twist()
    _acc = Twist()

    trans = Transform()

    pos.x = data.position.x
    pos.y = data.position.y
    pos.z = data.position.z


    vel.x = data.velocity.x
    vel.y = data.velocity.y
    vel.z = data.velocity.z

    ang.x = 0
    ang.y = 0
    ang.z = data.yaw_dot

    acc.x = data.acceleration.x
    acc.y = data.acceleration.y
    acc.z = data.acceleration.z
	
    trans.translation = pos
    trans.rotation.x = rot[0] 
    trans.rotation.y = rot[1]
    trans.rotation.z = rot[2]
    trans.rotation.w = rot[3]
    _vel.linear = vel
    _vel.angular = ang
    _acc.linear = acc

    multiDOF.transforms = [trans]
    multiDOF.velocities = [_vel]
    multiDOF.accelerations = [_acc]

    if zone != 3 and challenge_started:
        trajPub.publish(multiDOF)

def challenge_started_callback(data):
    global challenge_started
    challenge_started = data.data

def pos_to_MultiDOF():
    global trajPub, posePub, zone, challenge_started
    challenge_started = False
    zone = -1
    rospy.init_node("pos_to_multiDOF")

    rospy.Subscriber("/planning/pos_cmd", PositionCommand, convertCallback)
    rospy.Subscriber("/zone", Int32, zoneCallback)
    rospy.Subscriber("/red/challenge_started", Bool, challenge_started_callback)
    trajPub = rospy.Publisher("/red/position_hold/trajectory", MultiDOFJointTrajectoryPoint, queue_size=3)
    posePub = rospy.Publisher("/red/tracker/input_pose", PoseStamped, queue_size=3)

    rospy.spin()

if __name__ == "__main__":
    pos_to_MultiDOF()