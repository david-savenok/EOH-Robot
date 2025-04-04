from kin_planning import IKin
import numpy as np
import config
import matplotlib.pyplot as plt
from scipy.optimize import least_squares

h, L1, offset1, L2, offset2, L3, offset3, L4, offset4, L5, offset5, L6 = config.LENGTHS
home_position = config.ANGLES

COMj = [12, 12, 0]  # jengaX, jengaY, jengaZ (Loader)
blockHeight = 1  
numLayers = 9  
homePosition = [0, 12, 0]  

towerCenter = [-12, 12, 0]  
layerOrientation = True  

all_joint_angles = []
all_positions = []
last_angles = [-np.pi/2, -np.pi/2, np.pi/2, 0]  # Initial guess for the angles

def jengaIK(x, y, z, guess, motor6_angle=0):
    bounds = [(-np.pi, np.pi), (-41*np.pi/36, 5*np.pi/36), (-np.pi, np.pi), (-np.pi, np.pi)]
    
    r_target = np.sqrt(x**2 + y**2)
    phi_target = np.arctan2(y, x)
    
    def func(thetas):
        theta1, theta2, theta3, theta4 = thetas
        r_guess = (L1 + 
                   L2 * np.cos(-theta2) +
                   L3 * np.cos(-theta2 + theta3) +
                   (L5 + offset4) * np.cos(-theta2 + theta3 - theta4))
        phi_guess = theta1 + np.arctan((offset3 + L4) / r_guess)
        z_guess = (h +
                   L2 * np.sin(-theta2) +
                   L3 * np.sin(-theta2 + theta3) +
                   (L5 + offset4) * np.sin(-theta2 + theta3 - theta4))
        
        orientation_residual = (-theta2 + theta3 - theta4)
        return np.array([r_guess - r_target, z_guess - z, phi_guess - phi_target, orientation_residual])
    
    def residual(thetas):
        return func(thetas)
    
    result = least_squares(residual, guess, bounds=np.array(bounds).T)
    joint_angles = result.x  
    
    return joint_angles  

def control_orientation_for_layer(layer):
    if layer % 2 == 0:
        motor6_angle = np.pi / 2  
    else:
        motor6_angle = 0    
    return motor6_angle

def create_command(joint_angle_sets):
    command = "/S*H*"
    for i, joint_set in enumerate(joint_angle_sets):
        if isinstance(joint_set, str):
            command += f"*/{joint_set}"
        else:
            command += "/M*"
            for joint in joint_set:
                command += f"{round(joint, 4)},"

            while len(joint_set) < 6:
                joint_set.append(0)  
            command = command[:-1]  # Remove trailing comma
        
        if i < len(joint_angle_sets) - 1:
            command += "*/M*0,0,0,0,0,0"
        else:
            command += "*/E*X,"
            for theta in home_position:
                command += f"{round(theta, 4)},"
            command = command[:-1] + "*"
    return command

# Main loop for building the Jenga tower
for layer in range(numLayers):
    towerOffset = layer * blockHeight 
    layerOrientation = not layerOrientation  # Alternate orientation per layer

    for i in range(3): 
        if layerOrientation:
            blockPosition = [towerCenter[0] - 2 + i * 2, towerCenter[1], towerOffset]
        else:
            blockPosition = [towerCenter[0], towerCenter[1] - 2 + i * 2, towerOffset]

        positions = [
            [COMj[0], COMj[1], COMj[2] + 3],  # Approach above block
            [COMj[0], COMj[1], COMj[2]],       # Grab block
            [COMj[0], COMj[1], COMj[2] + 3],  # Lift block
            [0, COMj[1], towerOffset + 2],     # Intermediate position 
            [blockPosition[0], blockPosition[1], towerOffset + 2],  # Above placement
            [blockPosition[0], blockPosition[1], towerOffset],  # Place block
        ]

        motor6_angle = control_orientation_for_layer(layer)
        
        all_joint_angles.append("C*C")

        # Grab the block (move to the position and calculate the joint angles)
        for idx, pos in enumerate(positions[:2]): 
            guess = last_angles
            thetas = jengaIK(pos[0], pos[1], pos[2], guess, motor6_angle)
            all_joint_angles.append(thetas.tolist()) 
            all_positions.append(pos)
            last_angles = thetas

        all_joint_angles.append("C*C")

        # Lift the block
        guess = last_angles
        thetas = jengaIK(positions[2][0], positions[2][1], positions[2][2], guess, motor6_angle)
        all_joint_angles.append(thetas.tolist())
        all_positions.append(positions[2])
        last_angles = thetas

        # Move to the intermediate position
        guess = last_angles
        thetas = jengaIK(positions[3][0], positions[3][1], positions[3][2], guess, motor6_angle)
        all_joint_angles.append(thetas.tolist())
        all_positions.append(positions[3])
        last_angles = thetas

        # Move to the target placement position
        for idx, pos in enumerate(positions[4:]): 
            guess = last_angles
            thetas = jengaIK(pos[0], pos[1], pos[2], guess, motor6_angle)
            all_joint_angles.append(thetas.tolist()) 
            all_positions.append(pos)
            last_angles = thetas

        all_joint_angles.append("C*O" if layer % 2 == 0 else "C*C")  # Toggle claw action after each layer

def call():
    return create_command(all_joint_angles)

# Generate and print command
print(create_command(all_joint_angles))

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
all_positions = np.array(all_positions)
ax.plot(all_positions[:, 0], all_positions[:, 1], all_positions[:, 2], marker='o', linestyle='-', label='Robot Path')
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("Jenga Tower Building Path")
ax.legend()
plt.show()
