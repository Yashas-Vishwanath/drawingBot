# goal of this script is to vectorize the .pgm image into .svg for rhino

import cv2
import numpy as np
import matplotlib.pyplot as plt

# load the .pgm image
img = cv2.imread('captures/Maps/4may.pgm', cv2.IMREAD_GRAYSCALE)


####### load the resolution and origin from yaml file #######
# Open the YAML file
with open('captures/Maps/4may.yaml', 'r') as file:
    lines = file.readlines()

    # Initialize variables for resolution and origin
    resolution = None
    origin = None

    # Loop through the lines of the YAML file
    for line in lines:
        # Check if the line contains the resolution parameter
        if line.startswith("resolution:"):
            resolution = float(line.split(":")[1].strip())
        
        # Check if the line contains the origin parameter
        elif line.startswith("origin:"):
            origin_values = line.split(":")[1].strip().split(',')
            origin_values = [value.strip().replace('[', '').replace(']', '') for value in origin_values]
            origin = [float(value) for value in origin_values]
            # keep only the first two values in origin
            origin = origin[:2]

# Check if resolution and origin were successfully extracted
if resolution is not None:
    print("Resolution:", resolution)
else:
    print("Resolution not found or invalid.")

if origin is not None:
    print("Origin (m):", origin)
else:
    print("Origin not found or invalid.")

# calculate the origin in pixels by dividing the origin in meters by the resolution
origin_px = [int(origin[0] / resolution), int(origin[1] / resolution)]
print("Origin (px):", origin_px)


# calculate the width and height of the image in pixels
width_px = img.shape[1]
height_px = img.shape[0]
print("Width (px):", width_px)
print("Height (px):", height_px)

## POINT TO REMEMBER: origin in yaml file is calculated from the bottom left corner of the image
## but origin in opencv is calculated from the top left corner of the image

# convert the origin values suitable for opencv by subtracting the y value from the height of the image
origin_opencv = [-origin_px[0], height_px + origin_px[1]]
print("Origin (opencv):", origin_opencv)

#### draw a cross of 1m x 1m at the origin_opencv in red color ####
# calculate the value of 1m in pixels
one_meter_px = int(1 / resolution)
print("1m in pixels:", one_meter_px)

# calculate the length and thickness of the cross
cross_length = one_meter_px // 2
cross_thickness = 1

# calculate the coordinates of the cross
cross_x1 = origin_opencv[0] - cross_length
cross_y1 = origin_opencv[1]
cross_x2 = origin_opencv[0] + cross_length
cross_y2 = origin_opencv[1]
cross_x3 = origin_opencv[0]
cross_y3 = origin_opencv[1] - cross_length
cross_x4 = origin_opencv[0]
cross_y4 = origin_opencv[1] + cross_length

# draw the cross on the image
cross_color = (0, 0, 255)
cv2.line(img, (cross_x1, cross_y1), (cross_x2, cross_y2), cross_color, cross_thickness)
cv2.line(img, (cross_x3, cross_y3), (cross_x4, cross_y4), cross_color, cross_thickness)

# # display the image with the cross and close if key is pressed
# cv2.imshow('image', img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()


# threshold the image to create a binary image
_, binary_image = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

# apply morphological opening to remove small noise
kernel = np.ones((3,3), np.uint8)
binary_image = cv2.morphologyEx(binary_image, cv2.MORPH_OPEN, kernel)

# find the contours of the binary image
contours, _ = cv2.findContours(binary_image, cv2.CHAIN_APPROX_NONE, cv2.CHAIN_APPROX_SIMPLE)

# create a blank image to draw the contours on
canvas = np.zeros_like(img)

# draw the contours on the blank image
cv2.drawContours(canvas, contours, -1, (255, 255, 255), 1)

# # draw the cross on the canvas in red color
# cross_color = (0, 0, 255)
# cv2.line(canvas, (cross_x1, cross_y1), (cross_x2, cross_y2), cross_color, cross_thickness)
# cv2.line(canvas, (cross_x3, cross_y3), (cross_x4, cross_y4), cross_color, cross_thickness)

# view the canvas with the contours drawn on it
cv2.imshow('canvas', canvas)    # this is the image with the contours drawn on it
cv2.imshow('original', img)     # this is the original image
cv2.waitKey(0)
cv2.destroyAllWindows()


#### save the contours as a .svg file ####
# create a figure and axis
fig, ax = plt.subplots()

# loop through the contours
for contour in contours:
    # calculate the area of the contour
    contour_area = cv2.contourArea(contour)

    # specify minimum contour area to remove small noise
    min_contour_area = 0

    if contour_area > min_contour_area:
        # extract the x and y coordinates of the contour
        x = contour[:, 0, 0]
        y = contour[:, 0, 1]

        # close the contour by adding the first point to the end
        x = np.append(x, x[0])
        y = np.append(y, y[0])

        # plot the closed contour
        ax.plot(x, y, 'k')

# set the aspect of the plot to be equal
ax.set_aspect('equal')

# turn off the axis
ax.axis('off')

# invert the y-axis
ax.invert_yaxis()

# save the plot as an .svg file in the exports folder
plt.savefig('process_pgm/exports/4may_cross.svg', format='svg', bbox_inches='tight')

# display the plot
plt.show()
