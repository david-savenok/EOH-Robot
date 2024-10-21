from ForwardKinematics import calculateForwardKinematics
import numpy as np
import matplotlib.pyplot as plt
import config

#vars
height = config.HEIGHT
# Define your points in a NumPy array (shape: (n_points, 3))
T01, T02, T03, T04, T05, T06 = calculateForwardKinematics()

# Split the points into x, y, and z coordinates
x = np.array([0, 0, T01[0, 3], T02[0, 3], T03[0, 3], T04[0, 3], T05[0, 3], T06[0, 3]])
y = np.array([0, 0, T01[1, 3], T02[1, 3], T03[1, 3], T04[1, 3], T05[1, 3], T06[1, 3]])
z = np.array([0, height, T01[2, 3], T02[2, 3], T03[2, 3], T04[2, 3], T05[2, 3], T06[2, 3]])

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the points
ax.scatter(x, y, z, color='b', s=10)  # s is the size of points
ax.plot(x, y, z, color='r', linewidth=2)

# Set labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D Scatter Plot')

# Show the plot
plt.show()