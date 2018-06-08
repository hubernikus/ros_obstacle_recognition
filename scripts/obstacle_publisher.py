#!/usr/bin/env python

'''
obstacle publisher class

@author lukashuber
@date 2018-06-08

'''

import matplotlib.pyplot as plt

import rospy
from geometry_msgs.msg import PolygonStamped, Point32

import numpy as np
from math import pi, floor

import warnings

class obstaclePublisher():
    def __init__(self, a=[1,2,1], p=[1,1,1], x0=[0,0,0], th_r=[0,0,0], sf=1, sigma=1):
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
        rate = rospy.Rate(3) # Frequency

        # Create publishers
        elli_pub = rospy.Publisher('ellipse_out', PolygonStamped, queue_size=10)

        fig = plt.figure()

        ax_3d = fig.add_subplot(111, projection='3d')

        #ax_3d.plot_surface(self.x_obs[0,:], self.x_obs[1,:], self.x_obs[2,:])

        # Enter main loop
        while False: #not rospy.is_shutdown():
            # Loginfo concerning publishing
            rospy.loginfo("Publishing ellipse %s" % rospy.get_time())

            # Publish ellipse points
            ell_poly = PolygonStamped()
            ell_poly.header.stamp = rospy.Time.now()
            ell_poly.header.frame_id = 'world'
            ell_poly.polygon.points = [Point32(self.x_obs[i][0], self.x_obs[i][1], self.x_obs[i][2]) for i in range(len(self.x_obs))]
            
            elli_pub.publish(ell_poly)

            # Wait zzzz*
            rate.sleep()

            
    

    def draw_ellipsoid_centered(self, numPoints=50, a_temp = [0,0], draw_sfObs = False, x0=[0,0,0]):
        if self.dim == 2:
            theta = np.linspace(-pi,pi, num=numPoints)
            #numPoints = numPoints
        else:
            theta, phi = np.meshgrid(np.linspace(-pi,pi, num=numPoints),np.linspace(-pi/2,pi/2,num=floor(numPoints/2) ) ) #
            #numPoints = numPoints[0]*numPoints[1]
            numPoints = numPoints*floor(numPoints/2)
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
            #x_obs[2,:] = (np.zeros((x_obs[1,:].shape))).reshape((1, -1))
        else:
            # TODO --- next line probs wrong. Power of zero...
            
            #x_obs(1,:,n) = a(1,:).*cos(phi).*cos(theta);
            x_temp = a[0]*np.cos(phi)*np.cos(theta)
            x_obs[0,:] = x_temp.reshape((1,-1))
            print('temp', x_temp)
            # x_obs(2,:,n) = a(2,:).*sign(theta).*cos(phi).*(1 - 0.^(2.*p(3,:)) - cos(theta).^(2.*p(1,:))).^(1./(2.*p(2,:)));
            x_obs[1,:] = (np.copysign(a[1], theta)*(1 - np.cos(theta)**(2*p[0]))**(1./(2.*p[1]))).reshape((1,-1))
            #x_obs[1,:] =  (a[1]*np.copysign(1, (theta))*np.cos(phi)*(1 - 0 **(2*p[2]) - np.cos(theta)**(2*p[0])) ** (1/(2*p[1])) ).reshape((1, -1))
            print('temp', x_temp)

            # x_obs(3,:,n) = a(3,:).*sign(phi).*(1 - (sign(theta).*cos(phi).*(1 - 0.^(2.*p(3,:)) - cos(theta).^(2.*p(1,:))).^(1./(2.*p(2,:)))).^(2.*p(2,:)) - (cos(phi).*cos(theta)).^(2.*p(1,:))).^(1./(2.*p(3,:)));
            x_obs[2,:] = (a[2]*np.copysign(1,phi)*(1 - (np.copysign(1,theta)*np.cos(phi)*(1 - 0 ** (2*p[2]) - np.cos(theta)**(2*p[0]))**(1/(2**p[1])))**(2*p[1]) - (np.cos(phi)*np.cos(theta)) ** (2*p[0])) ** (1/(2*p[2])) ).reshape((1,-1))
        
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
        rotMatrix = np.array([[cos(th_r), -sin(th_r)],
                              [sin(th_r),  cos(th_r)]])
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
        a=[1,2,3]
        theta_r=[1,2,3]
        
        obstaclePublisher()
    except rospy.ROSInterruptException:
        pass