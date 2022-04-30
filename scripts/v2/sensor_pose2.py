#!/usr/bin/python

import rospy

import tf2_ros
import tf2_geometry_msgs

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Int32

def transform_pose(input_pose, from_frame, to_frame):

    # **Assuming /tf2 topic is being broadcasted
    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = input_pose
    pose_stamped.header.frame_id = from_frame
    pose_stamped.header.stamp = rospy.Time.now()

    try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(0.5))
        return output_pose_stamped

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise

def convertCallback(data):
    pose = Pose()
    pose.orientation.w = 1.0

    output = transform_pose(pose, "red/camera", "world")
    if zone != 3:
        sensorPosePub.publish(output)

def zoneCallback(data):
    global zone
    zone = data.data

def sensor_pose():
    global sensorPosePub, zone
    zone = -1

    rospy.init_node("sensor_pose")

    print("Waiting for TF2 to load")
    global tf_buffer
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(2.0)
    print("Sensor Pose Node Started")

    rospy.Subscriber("/red/odometry", Odometry, convertCallback)
    rospy.Subscriber("/zone", Int32, zoneCallback)
    sensorPosePub = rospy.Publisher("/red/sensor_pose", PoseStamped, queue_size=3)

    rospy.spin()

if __name__ == "__main__":
    sensor_pose()