from kin_planning import IKin
import numpy as np
import config
from scipy.optimize import least_squares

h, L1, offset1, L2, offset2, L3, offset3, L4, offset4, L5, offset5, L6 = config.LENGTHS
home_position = config.ANGLES

COMj = [12, 12, 0] #jengaX, jengaY, jengaZ (Loader)
blockHeight = 1 #in Height of jenga block
deltaZ = 3 #in
numBlocks = 27 #9 layers
homePosition = [0, 12, 0] #Intermediate position
towerCenter = [-12, 12, 0] #Center of tower

for block in range(numBlocks):
    towerOffset = np.ceil(block/3)*blockHeight
    points = []
    points.append([0, 0, -1, COMj[0], COMj[1], COMj[2]+deltaZ]) #Go to COMjenga with vertical offset
    points.append([0, 0, -1, COMj[0], COMj[1], COMj[2]]) #Go grab COMjenga
    points.append("CC") #Close claw
    points.append([0, 0, -1, COMj[0], COMj[1], COMj[2]+deltaZ]) #Go to COMjenga with vertical offset
    points.append([0, 0, -1, homePosition[0], homePosition[1], homePosition[2] + towerOffset]) #Home position accounting for vertical offset

def jengaIK(x, y, z, guess):
    bounds = [(-np.pi, np.pi), (-41*np.pi/36,5*np.pi/36,), (-np.pi, np.pi), (-np.pi, np.pi)]
    r_target = np.sqrt(x**2 + y**2)
    phi_target = np.arctan2(y,x)
    
    def func(thetas):
        theta1, theta2, theta3, theta4 = thetas
        r_guess = (L1 +
                   L2 * np.cos(-theta2) +
                   L3 * np.cos(-theta2 + theta3) +
                   (L5 + offset4) * np.cos(-theta2 + theta3 - theta4))
        phi_guess = theta1 + np.arctan((offset3 + L4)/r_guess)
        z_guess = (h +
                   L2 * np.sin(-theta2) +
                   L3 * np.sin(-theta2 + theta3) +
                   (L5 + offset4) * np.sin(-theta2 + theta3 - theta4))
        
        orientation_residual = (-theta2 + theta3 - theta4)
        return np.array([r_guess - r_target, z_guess - z, phi_guess - phi_target, orientation_residual])
    def residual(thetas):
        return func(thetas)
    result = least_squares(residual, guess, bounds=np.array(bounds).T)
    return result.x



