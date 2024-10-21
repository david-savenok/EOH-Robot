import ModernRobotics as MR
from ModernRobotics import calculate_forward_kinematics
import numpy as np
import matplotlib.pyplot as plt

# Define your points in a NumPy array (shape: (n_points, 3))
points = calculate_forward_kinematics(inputs)




# Split the points into x, y, and z coordinates
x = points[:, 0]
y = points[:, 1]
z = points[:, 2]

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