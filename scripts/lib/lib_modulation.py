'''
Obstacle Avoidance Library with different options

@author Lukas Huber
@date 2018-02-15

'''
import numpy as np
import numpy.linalg as LA

import sys 
lib_string = "/home/lukas/catkin_ws/src/obstacle_recognition/scripts/lib/"
if not any (lib_string in s for s in sys.path):
    sys.path.append(lib_string)
    
from lib_obstacleAvoidance import *

def obs_avoidance_convergence(x, xd, obs):
    # Initialize Variables
    N_obs = len(obs) #number of obstacles
    d = x.shape[0]

    # Initial magnitude - this is kept cosntant througout the algorithm
    xd_init_mag = np.sqrt(np.sum(xd**2))
    
    Gamma = np.zeros((N_obs))

    # Linear and angular roation of velocity
    xd_dx_obs = np.zeros((d,N_obs))
    xd_w_obs = np.zeros((d,N_obs)) #velocity due to the rotation of the obstacle

    E = np.zeros((d,d,N_obs))

    R = np.zeros((d,d,N_obs))
    M = np.eye(d)

    for n in range(N_obs):
        # rotating the query point into the obstacle frame of reference
        if obs[n].th_r: # Greater than 0
            R[:,:,n] = compute_R(d,obs[n].th_r)
        else:
            R[:,:,n] = np.eye(d)

        # Move to obstacle centered frame
        x_t = R[:,:,n].T.dot( (x-obs[n].x0))

        E[:,:,n], Gamma[n] = compute_basis_matrix(d,x_t,obs[n], R[:,:,n])

        if Gamma[n] < 1:
            print('WARNING -- gamma < 1')
            xd = -x_t
            xd = onstVel(xd, const_vel=xd_init_mag) # Input speed = output speed
            return xd

        # if Gamma[n]<0.99: 
        #     print(Gamma[n])
    w = compute_weights(Gamma,N_obs)

    #adding the influence of the rotational and cartesian velocity of the
    #obstacle to the velocity of the robot
    
    xd_obs = np.zeros((d))
    
    for n in range(N_obs):
    #     x_temp = x-np.array(obs[n].x0)
    #     xd_w_obs = np.array([-x_temp[1], x_temp[0]])*w[n]

        if d==2:                            #
            xd_w = np.cross(np.hstack(([0,0], obs[n].w)),
                            np.hstack((x-obs[n].x0,0)))
            xd_w = xd_w[0:2]
        else:
            xd_w = [] # TODO reactivate
            #xd_w = np.cross( obs[n].w, x-os[n].x0 )
            
        #xd_obs = xd_obs + w[n]*np.exp(-1/obs[n].sigma*(max([Gamma[n],1])-1))*  ( np.array(obs[n].xd) + xd_w )

        #xd_obs = xd_obs + w[n]* ( np.array(obs[n].xd) + xd_w )
        
        #the exponential term is very helpful as it help to avoid the crazy rotation of the robot due to the rotation of the object
    #xd = xd-xd_obs[n] #computing the relative velocity with respect to the obstacle
    xd_obs = np.array([0,0,0])
    xd = xd-xd_obs #computing the relative velocity with respect to the obstacle

    #ordering the obstacle number so as the closest one will be considered at
    #last (i.e. higher priority)
    # obs_order  = np.argsort(-Gamma)
    for n in range(N_obs):
        # if 'rho' in obs[n]:
        if hasattr(obs[n], 'rho'):
            rho = obs[n].rho
        else:
            rho = 1

    #     if isfield(obs[n],'eigenvalue')
    #         d0 = obs[n].eigenvalue
    #     else:
        d0 = np.ones((E.shape[1]-1))

        D = w[n]*(np.hstack((-1,d0))/abs(Gamma[n])**(1/rho))
        #     if isfield(obs[n],'tailEffect') && ~obs[n].tailEffect && xdT*R(:,:,n)*E(:,1,n)>=0 #the obstacle is already passed, no need to do anything
        #         D(1) = 0.0

        if D[0] < -1.0:
            D[1:] = d0
            if xd.T.dot( R[:,:,n]).dot( E[:,1,n]) < 0:
                D[0] = -1.0
        M = (R[:,:,n].dot( E[:,:,n]).dot( np.diag(D+np.hstack((1,d0)) )).dot( LA.pinv(E[:,:,n])).dot( R[:,:,n].T)).dot(M)
    xd = M.dot( xd) #velocity modulation 
    #if LA.norm(M*xd)>0.05:
    #    xd = LA.norm(xd)/LA.norm(M*xd)*M @xd #velocity modulation

    xd = xd + xd_obs # transforming back the velocity into the global coordinate system

        
    xd = constVel(xd, const_vel=xd_init_mag) # Input speed = output speed
    
    return xd


def compute_basis_matrix(d,x_t,obs, R):
    # For an arbitrary shape, the next two lines are used to find the shape segment
    th = np.arctan2(x_t[1],x_t[0])
    # if isfield(obs,.Tpartition.T):
    #     # TODO check
    #     ind = np.find(th>=(obs.partition(:,1)) & th<=(obs.partition(:,2)),1)
    # else:
    #     ind = 1
    
    ind = 1 # No partinioned obstacle
    #for ind in range(partitions)
    if hasattr(obs, 'sf'):
        a = np.array(obs.sf)*np.array(obs.a)
    elif hasattr(obs, 'sf_a'):
        #a = obs.a[:,ind] + obs.sf_a
        a = np.tile(obs.a, 2) + np.array(obs.sf_a)
    else:
        #a = obs.a[:,ind]
        a = np.array(obs.a)
        
    #p = obs.p[:,ind]
    p = np.array(obs.p)

    Gamma = np.sum((x_t/a)**(2.*p))

    # TODO check calculation
    nv = (2.*p/a*(x_t/a)**(2.*p - 1.)) #normal vector of the tangential hyper-plane

    E = np.zeros((d, d))
    
    if hasattr(obs,'center_dyn'): # automatic adaptation of center 
        #R= compute_R(d, obs.th_r)
        E[:,0] = - (x_t - R.T.dot( (np.array(obs.center_dyn) - np.array(obs.x0))) )

        #E(:,1) = - (x_t - (obs.x_center*obs.a))
        #elif 'x_center' in obs: # For relative center
    #    E[:,0] = - (x_t - (obs.x_center*obs.a))
    else:
        E[:,0] = - x_t

    E[1:d,1:d] = -np.eye(d-1)*nv[0]

    # Make diagonal to circle to improve behavior
    nv_hat = -x_t
    
    #generating E, for a 2D model it simply is: E = [dx [-dx(2)dx(1)]]
    E[0,1:d] = nv[1:d].T
    E[1:d,1:d] = -np.eye((d-1))*nv[0]

    # if d == 3:
    #     E[:,+1] = [0-nv(3)nv(2)]
    return E, Gamma

def constVel(xd, const_vel=0.3):
    
    xd_norm = np.sqrt(np.sum(xd**2))

    if xd_norm==0:
        return xd
    
    return xd/xd_norm*const_vel # 

