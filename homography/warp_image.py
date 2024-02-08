import cv2
import numpy as np

# Load the image
image = cv2.imread('captures/IMG20240205171318.jpg')  # Replace 'your_input_image.jpg' with your image file path

# Resize the image to a smaller resolution
resize_factor = 0.5  # Adjust the resize factor as needed
resized_image = cv2.resize(image, None, fx=resize_factor, fy=resize_factor)

# Convert the resized image to grayscale
gray = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)

# Define the ArUco dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_1000)

# Define parameters for marker detection
parameters = cv2.aruco.DetectorParameters()

# Detect ArUco markers
corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)


# load in the calibration data
calib_data_path = "distance_estimation/calib_data/MultiMatrix.npz"

calib_data = np.load(calib_data_path)
print(calib_data.files)

cam_mat = calib_data["camMatrix"]
dist_coef = calib_data["distCoef"]
r_vectors = calib_data["rVector"]
t_vectors = calib_data["tVector"]

# in centimeters (measure your printed marker size)
MARKER_SIZE = 7.93  # 7x7_1000 300pixels
# MARKER_SIZE = 10.57 # 7x7_1000 400pixels
# MARKER_SIZE = 13.20 # 7x7_1000 500pixels

rVec, tVec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, MARKER_SIZE, cam_mat, dist_coef)

# Convert rotation vectors to rotation matrices
rotation_matrices = [cv2.Rodrigues(rvec)[0] for rvec in rVec]

# # Reshape translation vectors to (N, 3, 1) for compatibility with rotation matrices
# tVec_reshaped = np.array([tvec.reshape((3, 1)) for tvec in tVec])

# # Concatenate rotation matrices with translation vectors
# extrinsics = np.concatenate((rotation_matrices, tVec_reshaped), axis=2)

# # Compute homography matrices
# homography_matrices = [cam_mat @ extrinsic for extrinsic in extrinsics]

# Concatenate rotation matrices without translation vectors
homography_matrices = [cam_mat @ rotation_matrix for rotation_matrix in rotation_matrices]

print(rotation_matrices[0].shape)
# print(tVec_reshaped[0].shape)
print(homography_matrices[0].shape)
print(homography_matrices[0].dtype)


# Apply homography transformation to the entire image
image_warped = cv2.warpPerspective(resized_image, homography_matrices[0], (resized_image.shape[1], resized_image.shape[0]))

# Display the original and warped images
cv2.imshow('Original Image', resized_image)
cv2.imshow('Warped Image', image_warped)
cv2.waitKey(0)
cv2.destroyAllWindows()










# Convert corner coordinates to integer values
# int_corners = np.intp(corners)

# # Define the coordinates of corners of the image to be warped subtracting 50 pixels from the corners
# top_left = np.array([50, 50])
# top_right = np.array([(resized_image.shape[1] - 50), 50])
# bottom_right = np.array([(resized_image.shape[1] - 50), (image.shape[0] - 50)])
# bottom_left = np.array([50, (resized_image.shape[0] - 50)])

# if ids is not None and len(ids) > 0:
#     # Draw circles at the corner points of the detected ArUco marker
#     for corner in corners[0][0]:
#         corner_int = (int(corner[0]), int(corner[1]))  # Convert floating-point coordinates to integers
#         cv2.circle(image, corner_int, 5, (0, 0, 255), -1)  # Red circles

# Display the resized image with circles drawn at corner points of the detected ArUco marker
#     cv2.imshow('ArUco Marker with Corner Points (Resized)', resized_image)
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()
# else:
#     print("No ArUco marker detected in the image.")





    # # Define the desired points for perspective correction (a rectangle)
    # desired_corners = np.array([top_left, top_right, bottom_right, bottom_left], dtype=np.float32)
    # print("desired corners of frame: ", desired_corners)
    # print("height: ", image.shape[0])
    # print("width: ", image.shape[1])

#     # display the resized image with the desired points for perspective correction
#     resize_factor = 0.3
#     resized_image = cv2.resize(image, None, fx=resize_factor, fy=resize_factor)
#     cv2.polylines(resized_image, [desired_corners.astype(int)], True, (0, 255, 0), 2)  # Green rectangle
#     cv2.imshow('Desired Points for Perspective Correction (Resized)', resized_image)
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()
# else:
#     print("No ArUco marker detected in the image.")
#     # the box goes out of the frame because image is resized

#     # Compute homography matrix using the detected ArUco marker's corners
#     homography_matrix, _ = cv2.findHomography(corners, desired_corners)

#     # Apply homography transformation to the entire image
#     image_warped = cv2.warpPerspective(image, homography_matrix, (image.shape[1], image.shape[0]))

#     # Display the original and warped images
#     cv2.imshow('Original Image', image)
#     cv2.imshow('Warped Image', image_warped)
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()
# else:
#     print("No ArUco markers detected in the image.")
