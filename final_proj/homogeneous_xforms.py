import numpy as np
from math import pi, cos, sin


def param_A(a, theta, d, alpha):
    '''
    Calculates the array A_i from DH parameters
    Assumes angles are in radians now
    '''
    
    ct = cos(theta)
    st = sin(theta)
    ca = cos(alpha)
    sa = sin(alpha)
    return np.array([
        [ct, -1*st*ca, st*sa, a*ct],
        [st, ct*ca, -1*ct*sa, a*st],
        [0, sa, ca, d],
        [0, 0, 0, 1]
    ])


# NOTE FOR FUTURE:
#  DH params may need to be re-calculated due to slight measurement mismatch in diagram that was used

# Calculated DH parameters for the arm we're using (leaving out the joint parameters)
#  In the order of (a, theta, d, alpha) for linkages 1-4
dh_base_params = np.array([
    [0,0,96.326,pi/2],
    [130.231,100.62*pi/180,0,0],
    [124,79.38*pi/180,0,0],
    [133.4,0,0,0]
])

def A(q, i):
    '''
    Calculate the homogeneous transform A_i given the associated joint value q_i
    Because all joints in the arm are reovlute, we can just add q to the theta
    parameter (params[1]) to take into account the 
    '''
    params = dh_base_params[i].copy()
    params[1] = params[1] + q
    return param_A(params[0],params[1],params[2],params[3])

def H(start, end, q):
    '''
    Constructs chain of homogeneous transforms
    Assumes that start < end
        If start >= end, returns identity matrix (may not be what is desired)
    q is a length-4 vector containing the joint values (since assuming 4 DOF arm)
    '''
    res = np.identity(4)
    if start < end:
        for i in range(start, end):
            res = np.matmul(res, A(q[i], i))
    else:
        for i in range(end, start):
            res = np.matmul(res, A(q[i], i))
        res = H_inv(res)

    return res

def H_inv(H_arr):
    '''
    Uses the definition of H_(-1) as:
    [R^-1       R^-1*d]
    [  0           1  ]
    '''
    R = H_arr[0:3,0:3]
    d = np.array([H_arr[0:3,3]])
    R_inv = np.transpose(R)
    res = np.concatenate([R_inv, np.matmul(R_inv, np.transpose(d))], 1)
    res = np.concatenate([res, np.array([[0,0,0,1]])])

def T(end, q):
    '''
    Shorthand for H(0, end, q)
    '''
    return H(0, end, q)

def d(M):
    '''
    Shorthand for extracting the positional location of a homogeneous xform matrix
    '''
    return M[0:3,3]

def R(M):
    '''
    Shorthand for extracting the rotation information of a homogeneous xform matrix
    '''
    return M[0:3,0:3]