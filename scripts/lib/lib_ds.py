import numpy as np

def linearAttractor_const(x, x0 = 'default', velConst=0.3, distSlow=0.01):
    # change initial value for n dimensions
    # TODO -- constant velocity // maximum velocity
    
    dx = x0-x
    dx_mag = np.sqrt(np.sum(dx**2))
    
    dx = min(velConst, dx_mag/distSlow*velConst)*dx

    return dx
 
