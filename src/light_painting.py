from ModernRobotics import *
import numpy as np
import cv2
import matplotlib.pyplot as plt
import config

lengths = config.LENGTHS
home_position = config.ANGLES
height = lengths[0]

def generate_contours(image):
    """Gets contours from the given image

    Parameters
    ----------
    image : np.ndarray
        The given image (after preprocessing)

    Returns
    -------
    simplified_contours
        a list of contours that generally describe the lines in the given image, in image coordinates
    """
    
    src = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(src, 100, 200)
    contours, hierarchy = cv2.findContours(edges, cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

    # Check if image is loaded fine
    if src is None:
        print ('Error opening image!')
        print ('Usage: hough_lines.py [image_name -- default ' + image + '] \n')
        return -1


    if src.dtype != "uint8":
        src = cv2.convertScaleAbs(src)
    elif len(src.shape) == 2:
        src = cv2.cvtColor(src, cv2.COLOR_GRAY2BGR)

    simplified_contours = []
    for contour in contours:
        epsilon = 0.003*cv2.arcLength(contour, True)
        approx_contour = cv2.approxPolyDP(contour, epsilon, True)
        simplified_contours.append(approx_contour)

    min_contour_area = 100  # Adjust this value
    filtered_contours = [cnt for cnt in simplified_contours if cv2.contourArea(cnt) > min_contour_area]
    image_with_contours = src.copy()

    cv2.imshow("Source", src)
    cv2.drawContours(image_with_contours, filtered_contours, -1, (0, 255, 0), 2) 
    cv2.imshow("Original Image with Contours", image_with_contours)
    cv2.imshow("Canny edges", edges)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    return filtered_contours

def resize_image(image, max_width, max_height):
    # Load the image
    if image is None:
        print("Error: Image not found!")
        return None

    # Get current dimensions
    height, width = image.shape[:2]
    
    # Compute the scaling factor
    scale = min(max_width / width, max_height / height)

    # Compute new dimensions
    new_width = int(width * scale)
    new_height = int(height * scale)

    # Resize the image
    resized_image = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_AREA)

    return resized_image

def i2space(image_point, image):
    dims = image.shape
    x = 5
    ix = image_point[0]
    iy = image_point[1]
    real_height = 10
    real_width = 10
    if dims[0] >= dims[1]:
        y = real_width*(ix/dims[0]) - 6*(dims[1]/dims[0])
        z = -real_height*(iy/dims[0]) + height + 13
    else:
        y = real_width*(ix/dims[1]) - 6
        z = -real_height*(iy/dims[1]) + (height + 9)*(dims[0]/dims[1])
    return [float(x), float(y), float(z)]

image_file = r"src/testImage2.jpeg"
max_height = 1024
max_width = 1024
image = cv2.imread(image_file)
resized_image = resize_image(image, max_height, max_width)
contours = generate_contours(resized_image)
point_set_set_3D = []
for contour in contours:
    point_set_set_3D.append([i2space(point[0], resized_image) for point in contour])

#Plotting to check
fig, ax = plt.subplots(figsize=(8, 6))
for set in point_set_set_3D:
    set = np.array(set)
    ax.plot(set[:, 1], set[:, 2])
ax.set_title("Penis Drawing")
ax.set_xlabel("y")
ax.set_ylabel("z")
ax.set_aspect('equal')
ax.grid(True) 
plt.show()

#Turn these into T vectors
transformation_matrix_set = []
for set in point_set_set_3D:
    transformation_matrix_set.append([VecTose3(np.array([1, 0, 0, point[0], point[1], point[2]])) for point in set])

h, L1, elbow2Mag, L2, elbow3Mag, L3, L4, L5, L6= lengths

M = np.array([[1,0,0,L4+L6],[0,1,0,-(L1+L2+L3+L5)],[0,0,1,h],[0,0,0,1]])

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

theta_list_set = []
true_list_set = []
for set in transformation_matrix_set:
    results = [IKinSpace(S, M, T, np.array([0, 0, 0, 0, 0, 0]), 0.02, 0.01) for T in set]
    theta_list_set.append([i[0] for i in results])
    true_list_set.append([i[1] for i in results])

IKerrors = 0
for i in true_list_set:
    IKerrors += (len(i) - np.count_nonzero(i))

def create_command(theta_list_set):
    command = "/S*H*"
    contour_commands = []
    theta_firsts = [contour[0] for contour in theta_list_set]
    theta_lasts = [contour[-1] for contour in theta_list_set]
    for contour in theta_list_set:
        contour_command = ""
        for thetas in contour:
            for theta in thetas:
                contour_command += str(round(theta, 2))
                contour_command += ","
        contour_commands.append(contour_command[:-1])
    
    for i in range(len(contour_commands)):
        contour_command = contour_commands[i]
        command += "/M*TIMESCALE,"
        command += contour_command
        if i < len(theta_firsts)-1:
            command += "*/L*0*/M*TIMESCALE,"
            for theta in theta_lasts[i]:
                command += str(round(theta, 2))
                command += ","
            for theta in theta_firsts[i+1]:
                command += str(round(theta, 2))
                command += ","
            command = command[:-1]
            command += "*/L*1*"
        else:
            command += "*/E*X,"
            for theta in home_position:
                command += str(round(theta, 2))
                command += ","
            command = command [:-1]
            command += "*"
    return command
    
print(create_command(theta_list_set))