#!/usr/bin/env python

#'''
#obstacle publisher class
#
#@author lukashuber
#@date 2018-06-08
#
#'''

# Custom libraries
import sys 
lib_string = "/home/lukas/catkin_ws/src/obstacle_recognition/scripts/lib/"
if not any (lib_string in s for s in sys.path):
    sys.path.append(lib_string)

from lib_modulation import *
from lib_ds import *

# ROS tools    
import rospy

from obstacle_recognition.msg import Obstacle # Custom message
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped, Vector3, Twist
from nav_msgs.msg import Path

import tf

# MATH 
import numpy as np
from math import pi, floor

import copy # copying of lists


class TrajectoryPlanner():
    def __init__(self):
        print('Start node')
        # Initialize node
        rospy.init_node('ellipse_publisher', anonymous=True)
        rate = rospy.Rate(200) # Frequency

        # Create publishers
        pub_vel = rospy.Publisher('lwr/joint_controllers/passive_ds_command_vel', Twist, queue_size=5)
        pub_orient = rospy.Publisher('lwr/joint_controllers/passive_ds_command_orient', Quaternion, queue_size=5)

        # Create subscriber
        sub_pose = rospy.Subscriber("lwr/ee_pose", Pose, self.callback_pose)
        sub_obs = rospy.Subscriber("obstacle1", Obstacle, self.callback)

        # Create tranform listener
        self.listener = tf.TransformListener()
        
        self.obs = [0]

        self.firstObstacle = False
        while not self.firstObstacle:
            rospy.sleep(0.1)  # Wait zzzz* = 1 # wait
        rospy.loginfo('Recieved obstacles.')

        self.pose_ee = Pose() # End effector pose

        self.firstPose = False
        while not self.firstObstacle:
            rospy.sleep(0.1)  # Wait zzzz* = 1 # wait
        rospy.loginfo('Recieved end effector pose.')

        # Get initial transformation
        rospy.loginfo('Getting tf.')
        awaitingTrafo=True
        while(awaitingTrafo): # Trafo obs1 position
            try:
                self.pos_ob1, self.quat_ob1 = self.listener.lookupTransform("/mocap_world", "/object_1/base1_link", rospy.Time(0))                   
                awaitingTrafo=False
            except:
                rospy.loginfo('Waiting for TRAFO at {}'.format(rospy.get_time()))
                rospy.sleep(0.1)  # Wait zzzz*

        awaitingTrafo=True    
        while(awaitingTrafo): # TRAFO robot position
            try: # Get transformation
                self.pos_rob, self.pos_rob = self.listener.lookupTransform("/mocap_world", "/lwr_7_link", rospy.Time(0))
                awaitingTrafo=False
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo('Waiting for TRAFO')
                rospy.sleep(0.1)  # Wait zzzz*
            
        awaitingTrafo=True
        while(awaitingTrafo): # Trafo obs2 position
            try:
                self.pos_ob2, self.quat_ob2 = self.listener.lookupTransform("/mocap_world", "/object_2/base_link", rospy.Time(0))                   
                awaitingTrafo=False
            except:
                rospy.loginfo("Waiting for TRAFO at {}".format(rospy.get_time()))
                rospy.sleep(0.5)  # Wait zzzz*
                
        rospy.loginfo("All TF recieved")

        # Variables for trajectory prediciton
        while not rospy.is_shutdown():
            try: # Get transformation
                self.pos_rob, self.quat_rob = self.listener.lookupTransform("/mocap_world", "/lwr_7_link", rospy.Time(0))                   
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo("No <</lwr_7_link>> recieved")
                #continue
            x0_lwr7 = np.array([0,0,0])+np.array(self.pos_rob)

            try: # Get transformation
                    self.pos_ob2, self.quat_ob2 = self.listener.lookupTransform( "/mocap_world", "/object_2/base_link", rospy.Time(0))                   
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo("No <<object2>> recieved")
                #continue
            x_attractor = np.array([0,0,0]) + np.array(self.pos_ob2) # Transform to lwr_7_link
            
            obs_roboFrame = copy.deepcopy(self.obs)  # TODO -- change, because only reference is created not copy...
            for n in range(len(self.obs)): # for all bostacles
                try: # Get transformation
                    self.pos_ob1, self.quat_ob1 = self.listener.lookupTransform("/mocap_world", "/object_1/base1_link", rospy.Time(0))                   
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.loginfo("No <<object1>> recieved")
                    #continue

                quat_thr = tf.transformations.quaternion_from_euler(self.obs[n].th_r[0],
                                                                    self.obs[n].th_r[1],
                                                                    self.obs[n].th_r[2])
                quat_thr_mocap = tf.transformations.quaternion_multiply(self.quat_ob1, quat_thr)

                th_r_mocap = tf.transformations.euler_from_quaternion(quat_thr_mocap)

                # TODO apply transofrm to x0
                #q_x0 = tf.transformations.quaternion_multiply( self.quat_ob1, np.hstack(( self.obs[n].x0, 0)))
                #q_x0 = tf.transformations.quaternion_multiply( np.hstack((self.obs[n].x0, 0)), self.quat_ob1)
                #obs_roboFrame[n].x0 = q_x0[0:3] + np.array(self.pos_ob1) # Transpose into reference frame
                obs_roboFrame[n].x0 = self.obs[n].x0 + np.array(self.pos_ob1) # Transpose into reference frame
                
                # TODOOOOOOOOOOOOO --- activate
                obs_roboFrame[n].th_r =  [th_r_mocap[0],
                                          th_r_mocap[1],
                                          th_r_mocap[2]]# Rotate into reference frame

                # pose_ob = PoseStamped() # Publish pose for verification
                # pose_ob.header.stamp = rospy.Time.now()
                # pose_ob.header.frame_id = '/mocap_world'
                # pose_ob.pose.orientation = Quaternion(quat_thr_mocap[0],
                                                      # quat_thr_mocap[1],
                                                      # quat_thr_mocap[2],
                                                      # quat_thr_mocap[3])
                
                # pose_ob.pose.position = Point(obs_roboFrame[n].x0[0],
                                              # obs_roboFrame[n].x0[1],
                                              # obs_roboFrame[n].x0[2])
                # pub_pos1.publish(pose_ob)
                
            traj = Path()

            
            q0 = Quaternion(1, 0, 0, 0) # Unit quaternion for trajectory

            x = x0_lwr7 # x for trajectory creating
            x_hat =  x0_lwr7 # x fro linear DS

            # print('WARNING -- ERROR in obs[n].w calculation')
            # print(obs_roboFrame[0])
            obs_roboFrame[n].center_dyn = obs_roboFrame[n].x0
            
            obs_roboFrame[0].w = [0,0,0]
            obs_roboFrame[0].xd = [0,0,0]
            
            
            #obs_roboFrame = []
            ds_init = linearAttractor_const(x, x0=x_attractor)
            ds_modulated = obs_avoidance_convergence(x, ds_init, obs_roboFrame)
            ds_modulated = maxVel(ds_modulated)

            vel = Twist()
            vel.linear = Vector3(ds_modulated[0],ds_modulated[1],ds_modulated[2])
            vel.angular = Vector3(0,0,0)
            pub_vel.publish(vel)

            quat = Quaternion(0,1,0,0)
            pub_orient.publish(quat)
            
            rospy.loginfo("Publishing control in <<mocap_world>> frame.")
            
            rate.sleep()

    def callback(self,msg):
        self.firstObstacle = True
        i = 0 # TODO add multiple obstacles
        self.obs[i] = msg

    def callback_pose(self,msg):
        self.firstPose = True
        self.pose_ee = msg

        
if __name__ == '__main__':
    try:
         TrajectoryPlanner()
    except rospy.ROSInterruptException:
        pass
 
