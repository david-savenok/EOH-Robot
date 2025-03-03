# -*- coding: utf-8 -*-
"""
Created on Mon Oct 14 17:42:01 2024

@author: apark
"""
import config
import numpy as np
from sympy import *
import scipy
from scipy.linalg import expm
from ModernRobotics import * 

lengths = config.LENGTHS
thetas = config.ANGLES
height = config.ANGLES

h, L1, elbow2Mag, L2, elbow3Mag, L3, L4, L5, L6= config.LENGTHS
theta1, theta2, theta3, theta4, theta5, theta6 = config.ANGLES

Mend = np.array([[1,0,0,L4+L6],[0,1,0,-(L1+L2+L3+L5)],[0,0,1,h],[0,0,0,1]])
M2 = np.array([[1,0,0,0],[0,1,0,-(L1)],[0,0,1,h],[0,0,0,1]])
M3 = np.array([[1,0,0,0],[0,1,0,-(L1+L2)],[0,0,1,h],[0,0,0,1]])
M4 = np.array([[1,0,0,0],[0,1,0,-(L1+L2+L3)],[0,0,1,h],[0,0,0,1]])
M5 = np.array([[1,0,0,L4],[0,1,0,-(L1+L2+L3)],[0,0,1,h],[0,0,0,1]])
M6 = np.array([[1,0,0,L4],[0,1,0,-(L1+L2+L3+L5)],[0,0,1,h],[0,0,0,1]])

M = np.array([Mend, M2, M3, M4, M5, M6])

w1 = np.array([0,0,1])
w2 = np.array([1,0,0])
w3 = np.array([-1,0,0])
w4 = np.array([1,0,0])
w5 = np.array([0,-1,0])
w6 = np.array([1,0,0])

q1 = np.array([0,0,0])
q2 = np.array([0,-L1,h])
q3 = np.array([0,-L1-L2,h])
q4 = np.array([0,-(L1+L2+L3),h])
q5 = np.array([L4,-(L1+L2+L3),h])
q6 = np.array([L4,-(L1+L2+L3+L5),h])

v1 = np.cross(q1,w1)
v2 = np.cross(q2,w2)
v3 = np.cross(q3,w3)
v4 = np.cross(q4,w4)
v5 = np.cross(q5,w5)
v6 = np.cross(q6,w6)

S1 = np.concatenate((w1, v1))
S2 = np.concatenate((w2, v2))
S3 = np.concatenate((w3, v3))
S4 = np.concatenate((w4, v4))
S5 = np.concatenate((w5, v5))
S6 = np.concatenate((w6, v6))

S = np.array([S1, S2, S3, S4, S5, S6]).T

def calculateForwardKinematics(M, S):
    Sb1 = VecTose3(S[0])
    Sb2 = VecTose3(S[1])
    Sb3 = VecTose3(S[2])
    Sb4 = VecTose3(S[3])
    Sb5 = VecTose3(S[4])
    Sb6 = VecTose3(S[5])
    
    e1 = expm(Sb1*theta1)
    e2 = expm(Sb2*theta2)
    e3 = expm(Sb3*theta3)
    e4 = expm(Sb4*theta4)
    e5 = expm(Sb5*theta5)
    e6 = expm(Sb6*theta6)
    
    T02 = e1 @ M[1]
    T03 = e1 @ e2 @ M[2]
    T04 = e1 @ e2 @ e3 @ M[3]
    T05 = e1 @ e2 @ e3 @ e4 @ M[4]
    T06 = e1 @ e2 @ e3 @ e4 @ e5 @ M[5]
    T07 = e1 @ e2 @ e3 @ e4 @ e5 @ e6 @ M[0]
    print(e1)
    #calculate elbows:
    elbow1 = np.array([0,0,0]) + np.array([0, 0, h])
    w2new = TransToRp(T02)[0]@w2
    elbow2 = T02[0:3, 3] + elbow2Mag*(w2new/np.linalg.norm(w2new))
    w3new = TransToRp(T03)[0]@w3
    elbow3 = T03[0:3, 3]#elbow3 is calculated by T03 for simplicity, T03 has an offset
    new_T03 = T03[0:3, 3] + (-elbow3Mag*(w3new/np.linalg.norm(w3new)))
    print(TransToRp(T02)[0]@w2)
    return (T02[0:3, 3], new_T03, T04[0:3, 3], T05[0:3, 3], T06[0:3, 3], T07[0:3, 3], elbow1, elbow2, elbow3)

calculateForwardKinematics()

