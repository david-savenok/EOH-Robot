from kin_planning import IKin
import numpy as np
import config

lengths = config.LENGTHS
home_position = config.ANGLES
height = lengths[0]

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



