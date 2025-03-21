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

h, L1, offset1, L2, offset2, L3, offset3, L4, offset4, L5, offset5, L6 = config.LENGTHS
theta1, theta2, theta3, theta4, theta5, theta6 = config.ANGLES


def calculateForwardKinematics(thetas):
    #theta1, theta2, theta3, theta4, theta5, theta6 = thetas
    M = np.array([[1,0,0,L1+L2+L3+offset4+L5],[0,1,0,offset1+L4+offset5+L6],[0,0,1,h],[0,0,0,1]])
    #M2 = np.array([[1,0,0,L1],[0,1,0,offset1],[0,0,1,h],[0,0,0,1]])
    #M3 = np.array([[1,0,0,L1+L2],[0,1,0,offset1-offset2],[0,0,1,h],[0,0,0,1]])
    #M4 = np.array([[1,0,0,L1+L2+L3],[0,1,0,offset1],[0,0,1,h],[0,0,0,1]])
    #M5 = np.array([[1,0,0,L1+L2+L3+offset4],[0,1,0,offset1+L4],[0,0,1,h],[0,0,0,1]])
    #M6 = np.array([[1,0,0,L1+L2+L3+offset4+L5],[0,1,0,offset1+L4+offset5],[0,0,1,h],[0,0,0,1]])

    #M = np.array([Mend, M2, M3, M4, M5, M6])

    w1 = np.array([0,0,1])
    w2 = np.array([0,1,0])
    w3 = np.array([0,-1,0])
    w4 = np.array([0,1,0])
    w5 = np.array([1,0,0])
    w6 = np.array([0,1,0])

    q1 = np.array([0,0,0])
    q2 = np.array([L1,offset1,h])
    q3 = np.array([L1+L2,offset1-offset2,h])
    q4 = np.array([L1+L2+L3,offset1,h])
    q5 = np.array([L1+L2+L3+offset4,offset1+L4,h])
    q6 = np.array([L1+L2+L3+offset4+L5,offset1+L4+offset5,h])

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

    return FKinSpace(M, S, thetas)

def IKin(screw, guess):
    M = np.array([[1,0,0,L1+L2+L3+offset4+L5],[0,1,0,offset1+L4+offset5+L6],[0,0,1,h],[0,0,0,1]])

    w1 = np.array([0,0,1])
    w2 = np.array([0,1,0])
    w3 = np.array([0,-1,0])
    w4 = np.array([0,1,0])
    w5 = np.array([1,0,0])
    w6 = np.array([0,1,0])

    q1 = np.array([0,0,0])
    q2 = np.array([L1,offset1,h])
    q3 = np.array([L1+L2,offset1-offset2,h])
    q4 = np.array([L1+L2+L3,offset1,h])
    q5 = np.array([L1+L2+L3+offset4,offset1+L4,h])
    q6 = np.array([L1+L2+L3+offset4+L5,offset1+L4+offset5,h])

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
    
    Sb1 = VecTose3(S[0])
    Sb2 = VecTose3(S[1])
    Sb3 = VecTose3(S[2])
    Sb4 = VecTose3(S[3])
    Sb5 = VecTose3(S[4])
    Sb6 = VecTose3(S[5])
    
    e1 = expm(Sb1*screw[0])
    e2 = expm(Sb2*screw[1])
    e3 = expm(Sb3*screw[2])
    e4 = expm(Sb4*screw[3])
    e5 = expm(Sb5*screw[4])
    e6 = expm(Sb6*screw[5])

    T = e1 @ e2 @ e3 @ e4 @ e5 @ e6 @ M
    
    e0 = 0.02
    evar = 0.01
    thetaList, error = IKinSpace(S, M, T, guess, e0, evar)
    return [thetaList, error]