#!/usr/bin/env python

#'''
#obstacle publisher class
#
#@author lukashuber
#@date 2018-06-08
#
#'''

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import rospy
from geometry_msgs.msg import PolygonStamped, Point32, PoseStamped, Pose, Point
from obstacle_recognition.msg import Obstacle
from matplotlib.patches import Polygon

import numpy as np
from math import pi, floor

import tf

import warnings

class obstaclePublisher():
    def __init__(self, a=[1,1,1], p=[1,1,1], x0=[0,0,0], th_r=[0,2,0], sf=1, sigma=1):
        # Initialize variables
        self.a=a
        self.p = p
        self.x0 = x0
        self.th_r = th_r
        self.sf = sf
        self.sigma = sigma

        # Dimension of 
        self.dim=len(x0)

        # Calculate rotation matrix
        self.rotMatrix = compute_rotMat(self.th_r, self.dim)

        # Create point cloud outline
        self.draw_ellipsoid_centered()

        # Initialize node
        rospy.init_node('ellipse_publisher', anonymous=True)
        rate = rospy.Rate(20) # Frequency

        # Create publishers
        elli_pub = rospy.Publisher('ellipse1_out', PolygonStamped, queue_size=5)
        obs_pub = rospy.Publisher('obstacle1', Obstacle, queue_size=5)
        br_obs_pub = tf.TransformBroadcaster()
        

        

        # Create listener
        pose_sub = rospy.Subscriber("object_1/pose", PoseStamped, self.callback)

        self.pose_ellipse = Pose()
        self.pose_ellipse.position = Point(0.5,0,0)

        self.timeOld = rospy.Time.now()
        
        # Enter main loop
        #while False:
        while not rospy.is_shutdown():
            # Loginfo concerning publishing
            rospy.loginfo("Publishing ellipse %s" % rospy.get_time())

            
            # Obstacle for algorithm
            ell_poly = PolygonStamped()
            ell_poly.header.stamp = rospy.Time.now()
            ell_poly.header.frame_id = 'object_1/base1_link'

            # Publish new frame to new frame
            x_obs = self.x_obs
            ell_poly.polygon.points = [Point32(self.x_obs[i][0], self.x_obs[i][1], self.x_obs[i][2]) for i in range(len(self.x_obs))]
            elli_pub.publish(ell_poly)

            # Obstacle for algorithm
            # TODO -- for constant shape, keep those factors out of loop
            obstacle = Obstacle()
            obstacle.header = ell_poly.header
            obstacle.x0 = self.pos_ellipse
            obstacle.th_r = self.th_r
            obstacle.a = self.a
            obstacle.gamma = self.gamma
            obstacle.sf = self.sf
            
            obstacle.th_r = [0,0,0]
            
            obs_pub.publish(obstacle)
            
            rate.sleep()  # Wait zzzz*

    def callback(self, msg): # Subscriber callback
        
        time = rospy.Time.now()
        
        pos_ellipse_old = self.pos_ellipse
        self.pos_ellipse = msg.pose.position
        
        dt = time-self.timeOld

        deltaX = np.array([self.pos_ellipse.x - pos_ellipse_old.x,
                           self.pos_ellipse.y - pos_ellipse_old.y,
                           self.pos_ellipse.z - pos_ellipse_old.z]
        dx_new = deltaX/dt
        
        k_dx = 0.6 # kalman filter constant
        self.dx = k_dx*self.dx + (1-k_dx)*dx_new


        th_r_old = self.th_r
        deltaTH = tf.transformations.euler_from_quaternion(msg.pose.orientation)
        
        #deltaTH = getEulerZYX
        for ii in range(3):
            if deltaTH[ii] > pi:
                deltaTH[ii] = 2*pi - deltaTH[ii]
        w_new = deltaTH/dt

        k_w = 0.6 # kalman filter constant
        self.w = k_w*self.w + (1-k_w)*w_nw

        self.pose
        self.timeOld = time


    def draw_ellipsoid_centered(self, numPoints=32, a_temp = [0,0], draw_sfObs = False, x0=[0,0,0]):
        self.numPoints = numPoints 
        if self.dim == 2:
            theta = np.linspace(-pi,pi, num=numPoints)
            #numPoints = numPoints
        else:
            theta, phi = np.meshgrid(np.linspace(-pi,pi, num=numPoints),np.linspace(-pi/2,pi/2,num=floor(numPoints/2) ) ) #
            #numPoints = numPoints[0]*numPoints[1]
            numPoints = numPoints*floor(numPoints/2)
            self.resolution = numPoints
            theta = theta.T
            phi = phi.T

        # For an arbitrary shap, the next two lines are used to find the shape segment
        if hasattr(self,'partition'):
            warnings.warn('Partitions not implemented')
            for i in range(self.partition.shape[0]):
                ind[i,:] = self.theta>=(self.partition[i,1]) & self.theta<=(self.partition[i,1])
                [i, ind]=max(ind)
        else:
            ind = 0
            
        #a = obs[n].a[:,ind]
        #p = obs[n].p[:,ind]

        # TODO -- add partition index
        if sum(a_temp) == 0:
            a = self.a
        else:
#            import pdb; pdb.set_trace() ## DEBUG ##
            a = a_temp
            
        p = self.p[:]

        R = np.array(self.rotMatrix)

        x_obs = np.zeros((self.dim,numPoints))
        
        if self.dim == 2:
            x_obs[0,:] = (a[0]*np.cos(theta)).reshape((1,-1))
            x_obs[1,:] = (np.copysign(a[1], theta)*(1 - np.cos(theta)**(2*p[0]))**(1./(2.*p[1]))).reshape((1,-1))
            print('wrong dim')
                
        else:
            # TODO --- next line probs wrong. Power of zero...
            x_obs[0,:] = (a[0]*np.cos(phi)*np.cos(theta)).reshape((1,-1))
            x_obs[1,:] = (a[1]*np.copysign(1., theta)*np.cos(phi)*(1. - np.cos(theta)**(2.*p[0]))**(1./(2.*p[1]))).reshape((1,-1))

            #x2_a = a[2]*np.copysign(1,phi)
            #x2_b = (1 - (np.copysign(1,theta)*np.cos(phi)*(1 - 0 ** (2*p[2]) - np.cos(theta)**(2*p[0]))**(1/(2**p[1])))**(2*p[1]) - (np.cos(phi)*np.cos(theta)) ** (2*p[0]))
            #x2_c =  (1./(2*p[2]))

            #x2_dd = x2_a *x2_b**x2_c

            #x_obs[2,:] = (x2_a * x2_b).np.reshape((1,-1))
            x_obs[2,:] = (a[2]*np.copysign(1,phi)*(1. - (np.copysign(1,theta)*np.cos(phi)*(1. - 0. ** (2*p[2]) - np.cos(theta)**(2.*p[0]))**(1./(2.**p[1])))**(2.*p[1]) - (np.cos(phi)*np.cos(theta)) ** (2.*p[0])) ** (1./(2.*p[2])) ).reshape((1,-1))

            # print()
            # print('x_obs 1 ',x_obs[0,:])
            # print('x_obs 2 ',x_obs[1,:])
            # print('x_obs 3 ',x_obs[2,:])

            # print()
            # print()
            # print('x2_a',x2_a)
            # print('x2_b',x2_b)
            # print('x2_c',x2_c)
            # print('x2_d',x2_dd)
            # import pdb; pdb.set_trace()
        
        # TODO for outside function - only sf is returned, remove x_obs to speed up
        x_obs_sf = np.zeros((self.dim,numPoints))
        if hasattr(self, 'sf'):
            if type(self.sf) == int or type(self.sf) == float:
                x_obs_sf = R.dot(x_obs*self.sf) + np.tile(x0,(numPoints,1)).T
            else:
                x_obs_sf = R.dot(x_obs*np.tile(self.sf,(1,numPoints))) + np.tile(x0, (numPoints,1)).T 
        else:
            x_obs_sf = R.dot(fx_obs) + np.tile(x0,(1,numPoints))

        # TODO uncomment
        #x_obs = R.dot(x_obs) + np.tile(np.array([x0]).T,(1,numPoints))

        if sum(a_temp) == 0:
            self.x_obs = x_obs.T.tolist()
            self.x_obs_sf = x_obs_sf
        else:
             return x_obs_sf
        
        #self.x_obs_sf = R @x_obs_sf.T.tolist()

def compute_rotMat(th_r=0, d=3):
    if th_r == 0:
        rotMatrix = np.eye(d)
        return rotMatrix

    # rotating the query point into the obstacle frame of reference
    if d == 2 :
        rotMatrix = np.array([[np.cos(th_r), -np.sin(th_r)],
                              [np.sin(th_r),  np.cos(th_r)]])
    elif d == 3:
        # Use quaternions?!
        R_x = np.array([[1, 0, 0,],
                        [0, np.cos(th_r[0]), np.sin(th_r[0])],
                        [0, -np.sin(th_r[0]), np.cos(th_r[0])] ])

        R_y = np.array([[np.cos(th_r[1]), 0, -np.sin(th_r[1])],
                        [0, 1, 0],
                        [np.sin(th_r[1]), 0, np.cos(th_r[1])] ])

        R_z = np.array([[np.cos(th_r[2]), np.sin(th_r[2]), 0],
                        [-np.sin(th_r[2]), np.cos(th_r[2]), 0],
                        [ 0, 0, 1] ])

        rotMatrix = R_x.dot(R_y).dot(R_z)
    else:
        print('rotation not yet defined in dimensions d>3')
        return np.eye(self.d)
    
    return rotMatrix

        
def talker():
    return 0
 
if __name__ == '__main__':
    try:
        a=[0.35,0.10,0.10]
        th_r=[0,0,0]
        p=[10,1,1]
        
        
        obstaclePublisher(a=a, th_r=th_r, p=p)
    except rospy.ROSInterruptException:
        pass
 
