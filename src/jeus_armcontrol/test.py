import numpy as np 
from numpy import arctan2, cos, sin, tan, pi

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
PI = np.pi; 
PI_TWO = 2* np.pi;
PI_DIV_2 = np.pi/2
PI_DIV_4 = np.pi/4

x = rotx(PI_DIV_2)
y = roty(PI_DIV_2*-1)
z = rotz(PI_DIV_2)

p =  np.array([2,1,3])
print(p)
print(x)
tp = x@p
print(tp)
tp = y@p
print(tp)
tp = z@p
print(tp)