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
        #ARMAGEDDON CODE epsilon = 0.001*cv2.arcLength(contour, True)
        approx_contour = cv2.approxPolyDP(contour, epsilon, True)
        simplified_contours.append(approx_contour)

    min_contour_area = 100  # Adjust this value
    filtered_contours = [cnt for cnt in simplified_contours if cv2.contourArea(cnt) > min_contour_area]
    
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    mask = np.zeros_like(gray, dtype=np.uint8)    
    cv2.drawContours(mask, contours, -1, (255), thickness=10)  # Increase thickness if needed
    mask = cv2.dilate(mask, np.ones((5,5), np.uint8), iterations=2)
    for contour in contours:
        cv2.drawContours(mask, [contour], -1, (0), thickness=cv2.FILLED)
    inpainted = cv2.inpaint(image, mask, inpaintRadius=3, flags=cv2.INPAINT_TELEA)
    colors = []
    for contour in filtered_contours:
        contour_colors = []
        red = 0
        green = 0
        blue = 0
        for point in contour:
            point = point[0]#Points seem to be double wrapped for some reason
            numbers = inpainted[point[1], point[0]].astype(np.int32)
            red = red + numbers[0]
            green += numbers[1]
            blue += numbers[2]
            contour_colors.append(numbers)
        
        colors.append([int(red/len(contour)), int(green/len(contour)), int(blue/len(contour))])

    image_with_contours = src.copy()

    cv2.imshow("Source", image)
    cv2.drawContours(image_with_contours, filtered_contours, -1, (0, 255, 0), 2) 
    cv2.imshow("Original Image with Contours", image_with_contours)
    cv2.imshow("Inpainted", inpainted)
    #cv2.imshow("Inpainting Mask", mask)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    return filtered_contours, colors

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
    real_height = 4
    real_width = 4
    if dims[0] >= dims[1]:
        x = real_width*(ix/dims[0]) + (L1 + 7)*(dims[1]/dims[0])
        z = -real_height*(iy/dims[0]) + h + 2
    else:
        x = real_width*(ix/dims[1]) + L1+10
        z = -real_height*(iy/dims[1]) + (h + 4)*(dims[0]/dims[1])
    return [float(x), float(z)]

def lightPaintingIK(x,z, guess):
    bounds = [(-np.pi/6,np.pi/2), (-3*np.pi/4, 3*np.pi/4), (-np.pi, np.pi)]
    def func(thetas):
        theta2, theta3, theta4 = thetas
        x_guess = L1+(L2*np.cos(theta2))+L3*(np.cos(theta2-theta3))+2.5*(np.cos(theta2-theta3+theta4))
        z_guess = h+L2*np.sin(theta2)+L3*np.sin(theta2-theta3)+2.5*np.sin(theta2-theta3+theta4)
        return np.array([x_guess - x, z_guess - z, 0])
    def residual(thetas):
        return func(thetas)
    result = least_squares(residual, guess, bounds=np.array(bounds).T)
    return result.x

"""
ARMAGEDDON CODE
def lightPaintingIK(x,z):
    D = (x**2 + z**2 - L2**2 - L3**2) / (2 * L2 * L3)

    if abs(D) > 1:
        raise ValueError("Target is out of reach")

    theta3 = -np.arccos(D)  # elbow-down
    k1 = L1 + L2 * np.cos(theta3)
    k2 = L2 * np.sin(theta3)

    theta2 = np.arctan2(z, x) - np.arctan2(k2, k1)
    return theta2, theta3
"""

image_file = r"src/testImage8.jpeg"
max_height = 1024
max_width = 1024
image = cv2.imread(image_file)
resized_image = resize_image(image, max_height, max_width)
contours, contour_colors = generate_contours(resized_image)
point_set_set_3D = []
for contour in contours:
    point_set_set_3D.append([i2space(point[0], resized_image) for point in contour])

#Plotting to check
fig, ax = plt.subplots(figsize=(8, 6))
for set in point_set_set_3D:
    set = np.array(set)
    ax.plot(set[:, 0], set[:, 1])
ax.set_title("Drawing")
ax.set_xlabel("x")
ax.set_ylabel("z")
ax.set_aspect('equal')
ax.grid(True) 
plt.show()

#Turn these into T vectors
theta_list_set = []
true_list_set = []
results = []
last = [np.pi/4,0,0]
for set in point_set_set_3D:
    temp=[]
    for point in set:
        theta = lightPaintingIK(point[0], point[1], np.array(last))
        temp.append([0,theta[0], theta[1], 0, theta[2], 0])
        last = theta
    theta_list_set.append(temp)

#Checking the actual angles please
def plot_points():
    theta2_new = []
    theta3_new = []
    theta4_new = []
    x_new = []
    z_new = []
    for i in theta_list_set:
        for thetas_list in i:
            theta2_new.append(thetas_list[1])
            theta3_new.append(thetas_list[2])
            theta4_new.append(thetas_list[4])
    for i in range(len(theta2_new)):
        x_new.append(L1+(L2*np.cos(theta2_new[i]))+L3*(np.cos(theta2_new[i]-theta3_new[i]))+2.5*(np.cos(theta2_new[i]-theta3_new[i]+theta4_new[i])))
        z_new.append(h+L2*np.sin(theta2_new[i])+L3*np.sin(theta2_new[i]-theta3_new[i])+2.5*np.sin(theta2_new[i]-theta3_new[i]+theta4_new[i]))
    plt.plot(x_new, z_new, color='blue', alpha=1.0)
    plt.show()

IKerrors = 0
for i in true_list_set:
    IKerrors += (len(i) - np.count_nonzero(i))
#print(IKerrors)

def create_command(theta_list_set):
    command = "/S*H*"
    contour_commands = []
    light_colors = []
    theta_firsts = [contour[0] for contour in theta_list_set]
    theta_lasts = [contour[-1] for contour in theta_list_set]
    for contour in theta_list_set:
        contour_command = ""
        for thetas in contour:
            for theta in thetas:
                contour_command += str(round(theta*180/np.pi, 2))
                contour_command += ","
        contour_commands.append(contour_command[:-1])
    
    for color in contour_colors:
        light_color = ""
        for value in color:
            light_color += str(value)
            light_color += ","
        light_colors.append(light_color[:-1])

    for i in range(len(contour_commands)):
        contour_command = contour_commands[i]
        command += "*/L*"
        command += light_colors[i]
        command += "*/M*"
        command += contour_command
        if i < len(theta_firsts)-1:
            command += "*/L*0,0,0*/M*"
            for theta in theta_lasts[i]:
                command += str(round(theta, 2))
                command += ","
            for theta in theta_firsts[i+1]:
                command += str(round(theta, 2))
                command += ","
            command = command[:-1]
        else:
            command += "*/M*0,0,0,0,0,0"
            command += "*/E*X,"
            for theta in home_position:
                command += str(round(theta, 2))
                command += ","
            command = command [:-1]
            command += "*"
    return command

def call():
    return create_command(theta_list_set) + '/'

print(call())
plot_points()