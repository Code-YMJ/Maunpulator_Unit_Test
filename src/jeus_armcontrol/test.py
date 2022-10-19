import numpy as np 
from numpy import arctan2, cos, sin, tan, pi
from dynamixel_sdk  import *

def rotx(ang: float, unit='rad'):
    if unit == 'deg':
        ang = np.deg2rad(ang)

    return np.array([[1,        0,         0],
                     [0, cos(ang), -sin(ang)],
                     [0, sin(ang),  cos(ang)]])

def roty(ang: float, unit='rad'):
    if unit == 'deg':
        ang = np.deg2rad(ang)

    return np.array([[cos(ang),  0,  sin(ang)],
                     [0,  1,         0],
                     [-sin(ang),  0,  cos(ang)]])

def rotz(ang: float, unit='rad'):
    if unit == 'deg':
        ang = np.deg2rad(ang)

    return np.array([[cos(ang), -sin(ang),  0],
                     [sin(ang),  cos(ang),  0],
                     [0,         0,  1]])
# PI = np.pi; 
# PI_TWO = 2* np.pi;
# PI_DIV_2 = np.pi/2
# PI_DIV_4 = np.pi/4

# x = rotx(PI/3)
# x_ = rotx(-PI/3)
# y = roty(PI_DIV_2*-1)
# z = rotz(PI_DIV_2)
# ix = np.linalg.inv(x)
# # print(ix)
# # print(x_)
# iy = np.linalg.inv(y)
# iz = np.linalg.inv(z)
# p =  np.array([5,1,8])
# transp = np.array([-1,2,3])
# p = p-transp
# print(p)
# print(x)
# tp = iz@ix@p
# print(tp)
a = np.array([10,-2,3,5])
b = abs(a)
c = max(b)


def point_to_base(pc, t, eul):
    """
    pc : position vector of object in camera coord
    t : position from base coord to camera coord written in base coord
    eul : rotation vector from base coord to camera coord written in base coord
    """

    R = rotx(eul[0]) @ roty(eul[1]) @ rotz(eul[2])

    T = np.eye(4)
    T[0:3,0:3] = R
    T[0:3,3] = t



    if 0:
        Tp = np.eye(4)
        Tp[0:3,3] = pc
        Tb = inv_tform(Tp) @ inv_tform(T)
        pb = Tb[0:3,3]
    else:
        pb = -(R.T @ t + pc)

    return pb

def inv_tform(T: np.ndarray):
    """ Compute invese matrix of a transformation matrix in SE(3)
    
    Inv T =  | R_T, -R_T *d |
             |   0,       1 |
    """
    RT = T[0:3, 0:3].T
    d = T[0:3, 3]

    res = np.eye(4)
    res[0:3, 0:3] = RT
    res[0:3, 3] = -RT @ d

    return res
