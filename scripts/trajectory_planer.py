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
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
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
        rate = rospy.Rate(2) # Frequency

        self.n_obs = 1
        # Create publishers
        pub_traj = rospy.Publisher('ds_finalTrajectory', Path, queue_size=5)
        
        pub_pos1 = rospy.Publisher('pose_obstacle', PoseStamped, queue_size=5)

        # Create listener
        pose_sub = [0,0]
        pose_sub[0] = rospy.Subscriber("obstacle0", Obstacle, self.callback_ob0)
        pose_sub[1] = rospy.Subscriber("obstacle1", Obstacle, self.callback_ob1)
        attr_sub = rospy.Subscriber("attr", Obstacle, self.callback_ob1)
        self.listener = tf.TransformListener() # TF listener

        self.n_obs = 1
        self.obs = [0]*self.n_obs
        self.pos_obs=[0]*self.n_obs
        self.quat_ob=[0]*self.n_obs

        print('wait obstacle')
        self.awaitObstacle = [True for i in range(self.n_obs)]
        while any(self.awaitObstacle):
            rospy.sleep(0.1)  # Wait zzzz* = 1 # wait
        print('got it')

        # Get initial transformation
        rospy.loginfo('Getting Transformationss.')
        awaitingTrafo_ee=True
        awaitingTrafo_obs[i]= [True] * self.obs    
        awaitingTrafo_attr=True

        for i in len(self.obs):
            while(awaitingTrafo_obs[i]): # Trafo obs1 position
                try:
                    self.pos_obs[i], self.quat_obs[i] = self.listener.lookupTransform("/mocap_world", "/object_{}/base_link".format(i), rospy.Time(0))                   
                    awaitingTrafo_obs[i]=False
                except:
                    rospy.loginfo('Waiting for TRAFO at {}'.format(rospy.get_time()))
                    rospy.sleep(0.1)  # Wait zzzz*

        while(awaitingTrafo_ee): # TRAFO robot position
            try: # Get transformation
                self.pos_rob, self.pos_rob = self.listener.lookupTransform("/mocap_world", "/lwr_7_link", rospy.Time(0))
                awaitingTrafo_ee=False
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo('Waiting for TRAFO')
                rospy.sleep(0.1)  # Wait zzzz*
            
        while(awaitingTrafo_attr): # Trafo obs2 position
            try:
                self.pos_attr, self.quat_attr = self.listener.lookupTransform("/mocap_world", "/object_2/base_link", rospy.Time(0))                   
                awaitingTrafo_attr=False
            except:
                rospy.loginfo("Waiting for TRAFO at {}".format(rospy.get_time()))
                rospy.sleep(0.5)  # Wait zzzz*
                
        rospy.loginfo("All TF recieved")

        self.iSim = 0

        # Variables for trajectory prediciton
        self.n_intSteps = 100
        self.dt = 0.05
        self.dim = 3 #  3D space

        while not rospy.is_shutdown():
            try: # Get transformation
                self.pos_rob, self.quat_rob = self.listener.lookupTransform("/mocap_world", "/lwr_7_link", rospy.Time(0))                   
            except:
                rospy.loginfo("No <</lwr_7_link>> recieved")
                #continue
            x0_lwr7 = np.array([0,0,0])+np.array(self.pos_rob)

            try: # Get transformation
                    self.pos_attr, self.attr = self.listener.lookupTransform( "/mocap_world", "/object_1/base_link", rospy.Time(0))                   
            except:
                rospy.loginfo("No <<object2>> recieved")
                #continue
            x_attractor = np.array([0,0,0]) + np.array(self.pos_attr) # Transform to lwr_7_link
            
            obs_roboFrame = copy.deepcopy(self.obs)  # TODO -- change, because only reference is created not copy...
            for n in range(len(self.obs)): # for all bostacles
                try: # Get transformation
                    self.pos_obs[n], self.quat_obs[n] = self.listener.lookupTransform("/mocap_world", "/object_{}/base_link".format(n), rospy.Time(0))                   
                except:# (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
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
                obs_roboFrame[n].x0 = self.ob[n].x0 + np.array(self.pos_ob[n]) # Transpose into reference frame
                
                # TODOOOOOOOOOOOOO --- activate
                obs_roboFrame[n].th_r =  [th_r_mocap[0],
                                          th_r_mocap[1],
                                          th_r_mocap[2]]# Rotate into reference frame

                pose_obs = PoseStamped() # Publish pose for verification
                pose_obs.header.stamp = rospy.Time.now()
                pose_obs.header.frame_id = '/mocap_world'
                pose_obs.pose.orientation = Quaternion(quat_thr_mocap[0],
                                                      quat_thr_mocap[1],
                                                      quat_thr_mocap[2],
                                                      quat_thr_mocap[3])
                
                pose_obs.pose.position = Point(obs_roboFrame[n].x0[0],
                                              obs_roboFrame[n].x0[1],
                                              obs_roboFrame[n].x0[2])
                pub_pos1.publish(pose_obs)
                
            traj = Path()
            traj.header.stamp = rospy.Time.now()
            traj.header.frame_id = '/mocap_world'
            
            q0 = Quaternion(1, 0, 0, 0) # Unit quaternion for trajectory

            x = x0_lwr7 # x for trajectory creating
            x_hat =  x0_lwr7 # x fro linear DS

            # print('WARNING -- ERROR in obs[n].w calculation')
            # print(obs_roboFrame[0])
            obs_roboFrame[n].center_dyn = obs_roboFrame[n].x0
            
            obs_roboFrame[0].w = [0,0,0]
            obs_roboFrame[0].xd = [0,0,0]
            
            #obs_roboFrame = []
            loopCount = 0
            for iSim in range(self.n_intSteps):
                ds_init = linearAttractor_const(x, x0=x_attractor)
                ds_modulated = obs_avoidance_convergence(x, ds_init, obs_roboFrame)
                
                x = ds_modulated*self.dt + x
                x_hat = ds_init*self.dt + x_hat
                #x=x_hat
                
                # Add to trajectory
                pose = PoseStamped()
                pose.header = traj.header
                pose.pose.position = Point(x[0],x[1],x[2])
                #pose.pose.position = Point(x_hat[0],x_hat[1],x_hat[2])
                pose.pose.orientation = q0
                traj.poses.append(pose)

                loopCount += 1
                
            pub_traj.publish(traj)
            rospy.loginfo("Publishing Trajectory %s" % rospy.get_time())
            
            self.iSim += 1
            
            rate.sleep()
        
        
    def callback_ob0(self,msg):
        i =0 # TODO add second obstacle
        self.awaitObstacle[i] = False
        self.obs[i] = msg

    def callback_ob1(self,msg):
        i =1 # TODO add second obstacle
        self.awaitObstacle[i] = False
        self.obs[i] = msg
 
if __name__ == '__main__':
    try:
         TrajectoryPlanner()
    except rospy.ROSInterruptException:
        pass
 
