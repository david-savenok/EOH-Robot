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
counter = 0 # Determining which block we are placing --> Left, Center, or Right in the set of three blocks per layer

for block in range(numBlocks):
    towerOffset = np.ceil(block/3)*blockHeight
    towerLeft = [towerCenter[0] - towerOffset, towerCenter[1] - towerOffset, 0]
    towerRight = [towerCenter[0] + towerOffset, towerCenter[1] + towerOffset, 0]
    points = []
    points.append([0, 0, -1, COMj[0], COMj[1], COMj[2]+deltaZ]) #Go to COMjenga with vertical offset
    points.append([0, 0, -1, COMj[0], COMj[1], COMj[2]]) #Go grab COMjenga
    points.append("CC") #Close claw
    if (counter < 2) : # Counter updates when the claw closes on a block, confirming a new block will be placed
        counter += 1   # Placements --> 0 places at Left; 1 places at Center; 2 places at Right
    else:
        counter = 0
    points.append([0, 0, -1, COMj[0], COMj[1], COMj[2]+deltaZ]) #Go to COMjenga with vertical offset
    points.append([0, 0, -1, homePosition[0], homePosition[1], homePosition[2] + towerOffset]) #Home position accounting for vertical offset
    if (counter == 1): # Move to and Lower the middle block in the layer
        points.append([0, 0, -1, towerCenter[0], towerCenter[1], homePosition[2] + towerOffset])
        points.append([0, 0, -1, towerCenter[0], towerCenter[1], towerOffset])
    elif (counter == 2): # Move to and Lower the right block in the layer
        points.append([0, 0, -1, towerRight[0], towerRight[1], homePosition[2] + towerOffset])
        points.append([0, 0, -1, towerRight[0], towerRight[1],towerOffset])
    else: # Move to and Lower the left block in the layer
        points.append([0, 0, -1, towerLeft[0], towerLeft[1], homePosition[2] + towerOffset])
        points.append([0, 0, -1, towerLeft[0], towerLeft[1], towerOffset])
    points.append("OC") #Open claw
    points.append([0, 0, -1, homePosition[0], homePosition[1], homePosition[2] + towerOffset]) #Return to Home Position



