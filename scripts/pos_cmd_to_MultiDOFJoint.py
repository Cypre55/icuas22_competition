#!/usr/bin/python

import rospy

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Transform

import tf.transformations

from quadrotor_msgs.msg import PositionCommand
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint

def convertCallback(data):

    # output = PoseStamped()
    # output.pose.position.x = data.position.x
    # output.pose.position.y = data.position.y
    # output.pose.position.z = data.position.z

    rot = tf.transformations.quaternion_from_euler(0, 0, data.yaw)

    # output.pose.orientation.x = rot[0]
    # output.pose.orientation.y = rot[1]
    # output.pose.orientation.z = rot[2]
    # output.pose.orientation.w = rot[3]

    # posePub.publish(output)

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

    trajPub.publish(multiDOF)

def pos_to_MultiDOF():
    # global PosCmd
    global trajPub, posePub

    rospy.init_node("pos_to_multiDOF")

    rospy.Subscriber("/planning/pos_cmd", PositionCommand, convertCallback)
    trajPub = rospy.Publisher("/red/position_hold/trajectory", MultiDOFJointTrajectoryPoint, queue_size=3)

    posePub = rospy.Publisher("/red/tracker/input_pose", PoseStamped, queue_size=3)

    # rate = rospy.Rate(20)

    # while not rospy.is_shutdown():
    #     if 
    #     rate.sleep() 

    rospy.spin()

if __name__ == "__main__":
    pos_to_MultiDOF()