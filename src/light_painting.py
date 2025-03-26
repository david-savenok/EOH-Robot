from kin_planning import IKin
import numpy as np
import cv2
import matplotlib.pyplot as plt
import config
from scipy.optimize import least_squares


h, L1, offset1, L2, offset2, L3, offset3, L4, offset4, L5, offset5, L6 = config.LENGTHS
home_position = config.ANGLES

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

    #cv2.imshow("Source", src)
    #cv2.drawContours(image_with_contours, filtered_contours, -1, (0, 255, 0), 2) 
    #cv2.imshow("Original Image with Contours", image_with_contours)
    #cv2.imshow("Canny edges", edges)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    
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
    ix = image_point[0]
    iy = image_point[1]
    real_height = 10
    real_width = 10
    if dims[0] >= dims[1]:
        x = real_width*(ix/dims[0]) + (L1+6)*(dims[1]/dims[0])
        z = -real_height*(iy/dims[0]) + h + 12
    else:
        x = real_width*(ix/dims[1]) + L1+6
        z = -real_height*(iy/dims[1]) + (h + 9)*(dims[0]/dims[1])
    return [float(x), float(z)]

def lightPaintingIK(x,z, guess):
    bounds = [(-3*np.pi/4,np.pi/4), (-3*np.pi/4, 3*np.pi/4), (-2*np.pi, 2*np.pi)]
    
    def func(thetas):
        theta2, theta3, theta4 = thetas
        x_guess = L1+(L2*np.cos(-theta2))+L3*(np.cos(-theta2+theta3))+(L5+offset4)*(np.cos(-theta2+theta3-theta4))
        z_guess = h+L2*np.sin(-theta2)+L3*np.sin(-theta2+theta3)+(L5+offset4)*np.sin(-theta2+theta3-theta4)
        return np.array([x_guess - x, z_guess - z, 0])
    def residual(thetas):
        return func(thetas)
    result = least_squares(residual, guess, bounds=np.array(bounds).T)
    return result.x


image_file = r"src/testImage3.jpeg"
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
    ax.plot(set[:, 0], set[:, 1])
ax.set_title("Penis Drawing")
ax.set_xlabel("x")
ax.set_ylabel("z")
ax.set_aspect('equal')
ax.grid(True) 
#plt.show()

#Turn these into T vectors
theta_list_set = []
true_list_set = []
results = []
last = [-np.pi/2,-np.pi/2,np.pi/2]
for set in point_set_set_3D:
    temp=[]
    for point in set:
        theta = lightPaintingIK(point[0], point[1], np.array(last))
        temp.append([0,theta[0], theta[1], theta[2], 0, 0])
        last = theta
    theta_list_set.append(temp)

IKerrors = 0
for i in true_list_set:
    IKerrors += (len(i) - np.count_nonzero(i))
#print(IKerrors)

def create_command(theta_list_set):
    command = "/S*H*"
    contour_commands = []
    theta_firsts = [contour[0] for contour in theta_list_set]
    theta_lasts = [contour[-1] for contour in theta_list_set]
    for contour in theta_list_set:
        contour_command = ""
        for thetas in contour:
            for theta in thetas:
                contour_command += str(round(theta*180/np.pi, 2))
                contour_command += ","
        contour_commands.append(contour_command[:-1])
    
    for i in range(len(contour_commands)):
        contour_command = contour_commands[i]
        command += "/M*"
        command += contour_command
        if i < len(theta_firsts)-1:
            command += "*/L*0*/M*"
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

def call_test():
    return create_command(theta_list_set) + '/'
print(call_test())