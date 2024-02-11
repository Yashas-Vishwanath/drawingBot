# goal of this script is to vectorize the .pgm image into .svg for rhino

import cv2
import numpy as np
import matplotlib.pyplot as plt

# load the .pgm image
img = cv2.imread('process_pgm/scans/playground.pgm', cv2.IMREAD_GRAYSCALE)
# img = cv2.resize(img, (0, 0), fx=5, fy=5) # only to view. dont use this for the actual vectorization

# threshold the image to create a binary image
_, binary_image = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

# apply morphological opening to remove small noise
kernel = np.ones((3,3), np.uint8)
binary_image = cv2.morphologyEx(binary_image, cv2.MORPH_OPEN, kernel)

# find the contours of the binary image
contours, _ = cv2.findContours(binary_image, cv2.CHAIN_APPROX_NONE, cv2.CHAIN_APPROX_SIMPLE)

# # create a color version of the original grayscale image
# img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

# # draw contours in bright red on the color image
# cv2.drawContours(img_color, contours, -1, (0, 0, 255), 1)

# create a blank image to draw the contours on
canvas = np.zeros_like(img)

# draw the contours on the blank image
cv2.drawContours(canvas, contours, -1, (255, 255, 255), 1)
# cv2.drawContours(canvas, contours, -1, (255, 255, 255), thickness=cv2.FILLED)

# view the canvas with the contours drawn on it
cv2.imshow('canvas', canvas)
cv2.imshow('original', img)
cv2.waitKey(0)
cv2.destroyAllWindows()

# # view the canvas with the contours drawn on it
# plt.imshow(canvas, cmap='gray', origin='upper')



##### save the contours in .svg using matplotlib #########

# specify minimum contour area to remove small noise
min_contour_area = 0

# create a new figure
fig, ax = plt.subplots()

# for contour in contours:
#     ax.plot(contour[:, 0, 0], contour[:, 0, 1], 'k')

for contour in contours:
    # calculate the area of the contour
    contour_area = cv2.contourArea(contour)

    if contour_area > min_contour_area:
        # extract the x and y coordinates of the contour
        x = contour[:, 0, 0]
        y = contour[:, 0, 1]

        # close the contour by adding the first point to the end
        x = np.append(x, x[0])
        y = np.append(y, y[0])

        # plot the closed contour
        ax.plot(x, y, 'k')

# set aspect ratio to be equal, turn off the axis
ax.set_aspect('equal')
ax.set_axis_off()

# invert the y-axis
ax.invert_yaxis()

plt.show()

plt.savefig('process_pgm/exports/playground.svg', bbox_inches='tight', pad_inches=0)

# # save the canvas as a vector image
# cv2.imwrite('process_pgm/exports/playground_fill.svg', canvas)
# cv2.imwrite('process_pgm/exports/playground_original.png', img)
# cv2.imwrite('process_pgm/exports/playground_binary.png', binary_image)