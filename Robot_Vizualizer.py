from ForwardKinematics import calculateForwardKinematics
import numpy as np
import matplotlib.pyplot as plt
import config

#vars
height = config.HEIGHT
# Define your points in a NumPy array (shape: (n_points, 3))
pos2, pos3, pos4, pos5, pos6, pos7, elbow1, elbow2, elbow3 = calculateForwardKinematics()

# Split the points into x, y, and z coordinates
x = np.array([0, elbow1[0], pos2[0], elbow2[0], pos3[0], elbow3[0], pos4[0], pos5[0], pos6[0], pos7[0]])
y = np.array([0, elbow1[1], pos2[1], elbow2[1], pos3[1], elbow3[1], pos4[1], pos5[1], pos6[1], pos7[1]])
z = np.array([0, elbow1[2], pos2[2], elbow2[2], pos3[2], elbow3[2], pos4[2], pos5[2], pos6[2], pos7[2]])
print(0, elbow1[0], pos2[0], elbow2[0], pos3[0], elbow3[0], pos4[0], pos5[0], pos6[0], pos7[0])
# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the points
ax.scatter(x, y, z, color='b', s=10)  # s is the size of points
ax.plot(x, y, z, color='r', linewidth=2)

ax.set_xlim([-10, 10])
ax.set_ylim([-20, 20])
ax.set_zlim([-10, 10])

# Set labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D Scatter Plot')

# Show the plot
plt.show()