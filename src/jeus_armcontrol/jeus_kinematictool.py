import numpy as np
from copy import deepcopy
from numpy import arctan2, cos, sin, tan, pi
from dataclasses import dataclass, field

# THRESHOLD =====================================\
TINY = 1e-6

# Math =====================================\
PI = np.pi; 
PI_TWO = 2* np.pi;
PI_DIV_2 = np.pi/2
PI_DIV_4 = np.pi/4

INF = np.inf

GRAV_ACC = -9.80665
DEFAULT_GRAV_ACC_VEC = np.array([0, 0, GRAV_ACC])

# UNIT CONVERSION ========================================
RAD2DEG = 180.0 / np.pi
DEG2RAD = np.pi / 180.0

DEG2RAD_POSE = np.array([1, 1, 1, DEG2RAD, DEG2RAD, DEG2RAD])
RAD2DEG_POSE = np.array([1, 1, 1, RAD2DEG, RAD2DEG, RAD2DEG])

DPS2RPM = 60/360
RPM2DPS = 360/60



METER2MM = 1000.0
MM2METER = 0.001

# --- ARMFORM
FRONT = 'FRONT'
REAR = 'REAR'
UP = 'UP'
DOWN = 'DOWN'
NONFLIP = 'NONFLIP'
FLIP = 'FLIP'

def get_link_tform_six_para(link, ang):
    """ Compute a tform matrix from link to link with six parameter
    """
    d = link[1]
    a = link[2]
    b = link[5]
    
    cos_th = cos(link[0]+ang); cos_al = cos(link[3]); cos_be = cos(link[4])
    sin_th = sin(link[0]+ang); sin_al = sin(link[3]); sin_be = sin(link[4])
    
    return np.array([[ cos_be*cos_th - sin_al*sin_be*sin_th, -cos_al*sin_th, sin_be*cos_th + cos_be*sin_al*sin_th, a*cos_th - b*cos_al*sin_th],
                     [ cos_be*sin_th + sin_al*sin_be*cos_th,  cos_al*cos_th, sin_be*sin_th - cos_be*sin_al*cos_th, a*sin_th + b*cos_al*cos_th],
                     [                       -cos_al*sin_be,         sin_al,                        cos_al*cos_be,               d + b*sin_al],
                     [                                    0,              0,                                    0,                          1]])

def tr2eulXYZ(R, TINY=1e-4):
    """ convert a 3x3 rotation matrix into 3x1 XYZ euler vector
    TODO:
    - Check SO(3) Functions
    """

    if R[0, 2] < 1 and R[0, 2] > -1:
        phi = np.arctan2(-R[1, 2], R[2, 2])
        theta = np.arctan2(R[0, 2], np.sqrt(R[0, 0]**2 + R[0, 1]**2))
        psi = np.arctan2(-R[0, 1], R[0, 0])

    elif np.abs(R[0, 2] + 1) < TINY:
        theta = -np.pi/2
        psi = 0.0
        phi = np.arctan2(-R[1, 0], R[1, 1])

    elif np.abs(R[0, 2]-1) < TINY:
        theta = np.pi/2
        phi = np.arctan2(R[1, 0], R[1, 1])
        psi = 0.0
    else:
        raise Exception("[{}]".format("tr2eulXYZ") + \
                        "R[0,2] is out of range (1, -1), R[0,2] :", R[0, 2])


    return np.array([phi, theta, psi])

@dataclass
class Poly5Coeff:
    a0:float = 0.0
    a1:float = 0.0
    a2:float = 0.0
    a3:float = 0.0
    a4:float = 0.0
    a5:float = 0.0
    
def Poly5Coefficient(t0, q0, v0, a0, t1, q1, v1, a1):
    TINY = 1e-6
    
    T = t1-t0
    h = q1 - q0
    
    if T < TINY:
        u = np.array([q0, 0, 0, 0, 0, 0])
        return u
    
    T2 = T**2
    T3 = T**3
    T4 = T**4
    T5 = T**5
    
    u = Poly5Coeff()
    u.a0 = q0;
    u.a1 = v0;
    u.a2 = 0.5*a0;
    u.a3 = 1/(2*T3)*(20*h-(8*v1+12*v0)*T-(3*a0-a1)*T2);
    u.a4 = 1/(2*T4)*(-30*h+(14*v1+16*v0)*T+(3*a0-2*a1)*T2);
    u.a5 = 1/(2*T5)*(12*h-6*(v1+v0)*T+(a1-a0)*T2);
    
    return u

def PathPlanner(s_in, s_out, t0 = 0.0, t1 = 1.0):
    v0 = 0.; a0 = 0.
    v1 = 0.; a1 = 0.
    U = []
    for i in range(len(s_in)):
        U.append(Poly5Coefficient(t0, s_in[i], v0, a0, t1, s_out[i], v1, a1)  )
    return U

def get_state_traj(t:float, t0:float, u:Poly5Coeff):
    
    T = t - t0
    T2 = T**2
    T3 = T**3
    T4 = T**4
    T5 = T**5
    
    a0, a1, a2, a3, a4, a5 = u.a0, u.a1, u.a2, u.a3, u.a4, u.a5
    
    th = a0 + a1*T + a2*T2 + a3*T3 + a4*T4 + a5*T5
    th_d = a1 + 2*a2*T + 3*a3*T2 + 4*a4*T3 + 5*a5*T4
    th_dd = 2*a2 + 6*a3*T2 + 12*a4*T2 + 20*a5*T3
    
    return th, th_d, th_dd

def trotx(ang: float, unit='rad'):
    if unit == 'deg':
        ang = np.deg2rad(ang)

    mat = np.eye(4)
    mat[0:3, 0:3] = rotx(ang, unit)
    return mat

def troty(angle: float, unit='rad'):
    if unit == 'deg':
        angle = np.deg2rad(angle)

    mat = np.eye(4)
    mat[0:3, 0:3] = roty(angle, unit)
    return mat

def trotz(angle: float, unit='rad'):
    if unit == 'deg':
        angle = np.deg2rad(angle)

    mat = np.eye(4)
    mat[0:3, 0:3] = rotz(angle, unit)
    return mat

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
def eulXYZ2r(eul):
    """ convert a 3x1 euler vector into 3x3 rotation matrix
    """
    rx = rotx(eul[0])
    ry = roty(eul[1])
    rz = rotz(eul[2])

    return rx @ ry @ rz

def eulXYZ2tr(eul):

    rx = trotx(eul[0])
    ry = troty(eul[1])
    rz = trotz(eul[2])

    return rx @ ry @ rz
    
def tr2pose(T):
    p = np.zeros(6)
    p[0:3] = T[0:3, 3]
    p[3:] = tr2eulXYZ(T[0:3, 0:3]) 
    
    return p

def pose2tr(vec):
    eul = vec[3:]
    pos = vec[0:3]

    T = eulXYZ2tr(eul)
    
    T[0:3, 3] = pos

    return T

