#!/usr/bin/env python

#'''
#obstacle publisher class
#
#@author lukashuber
#@date 2018-06-08
#
#'''


import sys 
lib_string = "/home/lukas/catkin_ws/src/obstacle_recognition/scripts/"
if not any (lib_string in s for s in sys.path):
    sys.path.append(lib_string)

import rospy

from obstacle_recognition.msg import Obstacle
from trajectory_msgs.msg import JointTrajectory

import numpy as np
from math import pi, floor

import tf

class TrajectoryPlanner():
    def __init__(self):
        # Initialize node
        rospy.init_node('ellipse_publisher', anonymous=True)
        rate = rospy.Rate(20) # Frequency

        # Create publishers
        pub_traj = rospy.Publisher('ds_finalTrajectory', JointTrajectory, queue_size=5)

        # Create listener
        pose_sub = rospy.Subscriber("object_2/pose", Obstacle, self.callback)
        listener = tf.TransformListener()

        self.obs = []

        while not rospy.is_shutdown():
            rospy.loginfo("Publishing ellipse %s" % rospy.get_time())

            # Variables for trajectory prediciton
            self.n_integration = 1000
            self.dt = 0.01
            self.dim = 3 #  3D space

            try: # Get transformation
                (trans,rot) = listener.lookupTransform('/object_1/base_link', '/lwr_7_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            # Is at origin of base_link 2 - therefore no initial pos
            x0_lwr7 = np.array([0,0,0])

            x_attractor = np.array([0,0,0])
            x_attractor = x_attractor + np.array(trans) # Transform to lwr_7_link
            
            traj = JointTrajectory()
            
            self.pos = np.array((self.dim, self.n_integration))
                    for iSim in range(self.n_intSteps):
                        ds_init = linearAttractor_const(x, x0=x_attractor)
                        ds_modulated = obs_avoidance_convergence(x, ds_init, self.obs)
                        x = ds_modulted*dt + x

                        traj.points.positions.x = x[0]
                        traj.points.positions.y = x[1]
                        traj.points.positions.z = x[2]
                        
        traj = JointTrajectory()
        traj.header.time = rospy.Time.now()
        traj.frame_id = '/lwr_7_link'

        pub_traj.publish(traj)
        
        
    def callback(self,msg):
        i =0 # TODO add second obstacle
        obs[i] = msg

        
def linearAttractor_const(x, x0 = 'default', velConst=6):
    # change initial value for n dimensions
    # TODO -- constant velocity // maximum velocity
    dim, M = x.shape

    if x0=='default':
        x0 = dim*[0]
    xd = - (x-np.tile(x0,M))
    
    return xd

            
 
if __name__ == '__main__':
    try:
         TrajectoryPlanner()
    except rospy.ROSInterruptException:
        pass
 
