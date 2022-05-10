#!/usr/bin/python

import rospy
import os
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose, Twist, Transform, Vector3
from std_msgs.msg import Int32,Bool
import rosnode
import tf.transformations
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint


POSE_SAMPLING_INTERVAL = 5
FUEL_TRIG_DUR_TIME = 1
FUEL_INIT_WAIT_TIME = 5
ZONE_BOUNDS = [[-12.5, -8], [-8, 1], [1, 12.5]]
FSM_RATE = 10.0
STAT_THRESH = 0.1
FUEL_HEYAWWW_WAIT_TIME = 10

class FSM():
    def __init__(self):
        # Data
        self.curr_pose = None
        self.last_sampled_pose = None
        self.last_sampled_time = 0

        self.challenge_started = False

        self.zone = -1

        self.trigger = PoseStamped()

        # FUEL States
        self.fuel_triggered = False
        self.fuel_killed = False
        self.fuel_inited = False
        self.fuel_crashed = False

        self.fuel_last_init_time = None
        self.fuel_last_trig_time = None
        self.fuel_last_heyawww_time = None
        self.fuel_trig_time = None

        # Drone States
        self.drone_static = False

        # Subscribers
        self.odom_sub = rospy.Subscriber("/red/odometry", Odometry, self.odomCallback)
        self.started_sub = rospy.Subscriber("/red/challenge_started", Bool, self.challengeStartedCallback)

        # Publishers
        self.trigger_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.zone_pub = rospy.Publisher('/zone', Int32, queue_size=1)
        self.pose_pub = rospy.Publisher('/red/tracker/input_pose', PoseStamped, queue_size=1)
        self.trajPub = rospy.Publisher("/red/position_hold/trajectory", MultiDOFJointTrajectoryPoint, queue_size=3)

        # Timers
        rospy.Timer(rospy.Duration(1.0/FSM_RATE), self.fsm)

        print("FSM initialized")
    
    def dist(self, x, y):
        return np.sqrt((x.x-y.x)**2+(x.y-y.y)**2+(x.z-y.z)**2)

    def odomCallback(self, odom):
        self.curr_pose = odom.pose.pose
        self.getZone()
        self.checkDroneStatic()
        if self.last_sampled_pose is None or rospy.get_time()-self.last_sampled_time>POSE_SAMPLING_INTERVAL:
            self.last_sampled_pose=self.curr_pose
            self.last_sampled_time=rospy.get_time()
            

    def getZone(self):
        x = self.curr_pose.position.x
        if x >= ZONE_BOUNDS[0][0] and x < ZONE_BOUNDS[0][1]:
            self.zone = 1
        elif x >= ZONE_BOUNDS[1][0] and x < ZONE_BOUNDS[1][1]:
            self.zone = 2
        elif x >= ZONE_BOUNDS[2][0] and x < ZONE_BOUNDS[2][1]:
            self.zone = 3
        self.zone_pub.publish(self.zone)


    def challengeStartedCallback(self, bool):
        self.challenge_started=bool.data
        if self.challenge_started:
            rospy.loginfo("Challenge Started")

    def checkDroneStatic(self):
        # TODO: Check if drone is static
        # isStatic, Static Duration
        if self.curr_pose is not None and self.last_sampled_pose is not None and self.fuel_last_trig_time is not None:
            if self.dist(self.curr_pose.position,self.last_sampled_pose.position) < STAT_THRESH:
                if rospy.get_time()-self.last_sampled_time>5 and rospy.get_time()-self.fuel_last_trig_time>5:
                    self.drone_static = True

    def checkFUELAlive(self):
        nodes = rosnode.get_node_names()
        if "/exploration_node" in nodes or self.fuel_last_init_time is None or rospy.get_time()-self.fuel_last_init_time>FUEL_INIT_WAIT_TIME:
            self.fuel_crashed = False
        else:
            self.fuel_crashed = True


    def startFUEL(self):
        rospy.loginfo("Starting FUEL")
        self.fuel_inited = True
        self.fuel_killed = False
        self.fuel_triggered = False
        self.fuel_crashed = False

        self.fuel_last_init_time = rospy.get_time()

        # os.system("gnome-terminal -- roslaunch exploration_manager icuas.launch init_x:="+str(self.curr_pose.position.x)+" init_y:="+str(self.curr_pose.position.y)+" init_z:="+str(self.curr_pose.position.z))


    def triggerFUEL(self):
        if self.fuel_trig_time is None:
            rospy.loginfo("Triggering FUEL")
            msg = PoseStamped()
            msg.pose.position.x = self.curr_pose.position.x+1
            msg.pose.position.y = self.curr_pose.position.y
            msg.pose.position.z = self.curr_pose.position.z
            msg.pose.orientation = self.curr_pose.orientation
            self.pose_pub.publish(msg)
            self.fuel_trig_time = rospy.get_time()
        if rospy.get_time()-self.fuel_trig_time<FUEL_TRIG_DUR_TIME:
            self.trigger_pub.publish(self.trigger)
        else:
            self.fuel_triggered = True
            self.fuel_trig_time = None
            self.fuel_last_trig_time = rospy.get_time()

    def killFUEL(self):
        rospy.loginfo("Killing FUEL")
        os.system("rosnode kill /exploration_node")
        os.system("rosnode kill /traj_server")
        os.system("rosnode kill /waypoint_generator")
        self.fuel_killed = True
        self.fuel_inited = False
        self.fuel_triggered = False
        
    def heyawww(self):
        if self.fuel_last_heyawww_time is not None and rospy.get_time()-self.fuel_last_heyawww_time<FUEL_HEYAWWW_WAIT_TIME:
            return
        rospy.loginfo("Heyawww!!!!!!!")
        self.fuel_last_heyawww_time = rospy.get_time()
        self.drone_static = False
        final_pose = Pose()
        final_pose = self.curr_pose
        rot = tf.transformations.euler_from_quaternion([final_pose.orientation.x, final_pose.orientation.y, final_pose.orientation.z, final_pose.orientation.w])
        rot = list(rot)
        rot[2] += np.pi/6
        orient = tf.transformations.quaternion_from_euler(rot[0], rot[1], rot[2])
        final_pose.orientation.x = orient[0]
        final_pose.orientation.y = orient[1]
        final_pose.orientation.z = orient[2]
        final_pose.orientation.w = orient[3]
        # final_msg = PoseStamped()
        # final_msg.pose = final_pose
        # self.pose_pub.publish(final_msg)

        multiDOF = MultiDOFJointTrajectoryPoint()

        pos = Vector3()
        ang = Vector3()


        _vel = Twist()
        _acc = Twist()

        trans = Transform()

        pos.x = final_pose.position.x
        pos.y = final_pose.position.y
        pos.z = final_pose.position.z

        trans.translation = final_pose.position
        trans.rotation = final_pose.orientation

        multiDOF.transforms = [trans]
        multiDOF.velocities = [_vel]
        multiDOF.accelerations = [_acc]

        self.trajPub.publish(multiDOF)


    def fsm(self, event=None):
        # self.checkFUELAlive()
        # if self.challenge_started:
        #     if self.zone == 1 or self.zone == 2:
        #         if self.fuel_crashed:
        #             rospy.logerr("FUEL Crashed")
        #             self.startFUEL()
        #         if self.drone_static:
        #             self.heyawww()
        #         if not self.fuel_triggered:
        #             self.triggerFUEL()
                
        #     else: # Zone 3
        #         if not self.fuel_killed:
        #             rospy.loginfo("Reached Zone 3")
        #             # self.killFUEL()
                
        # else: # Challenge not started
        #     if self.zone != 3 and not self.fuel_inited and self.curr_pose is not None:
        #         rospy.sleep(2)
        #         self.startFUEL()
        pass

        # States

        # challenge_started
        # zone
        # fuel_triggered
        # fuel_inited
        # fuel_killed
        # fuel_worked
        # drone_static / fuel_crashed / fuel_stuck
        # zone3exp_started
        # tag_detected
        # ball_released

def fsmNode():
    rospy.init_node("fsm")
    FSM()
    rospy.spin()

if __name__ == "__main__":
    fsmNode()