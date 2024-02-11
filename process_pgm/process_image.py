import cv2
import numpy as np


np.set_printoptions(threshold=np.inf)


img = cv2.imread('process_pgm/scans/playground.pgm', cv2.IMREAD_GRAYSCALE)


h, w = img.shape


x, y = np.meshgrid(np.arange(w), np.arange(h))


cord = np.column_stack((x.flatten(), y.flatten()))


color = img[cord[:,1], cord[:,0]]


a = cord
b = color
print(img.shape)

# # resize the image to make it larger
# img = cv2.resize(img, (0,0), fx=5, fy=5)
# print(img.shape)

cv2.imshow('image', img)
cv2.waitKey(0)
cv2.destroyAllWindows()

# # save the image to a file in jpg and png format
# cv2.imwrite('process_pgm/scans/playground.jpg', img)
# cv2.imwrite('process_pgm/scans/playground.png', img)


