import glob
import cv2
import numpy as np

board_size = (8, 6)
square_size = 0.023

left_points = []
right_points = []

# Set the termination criteria for corner detection
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Load the video
video = cv2.VideoCapture("calibration.mp4")

# Check if the video was successfully loaded
if not video.isOpened():
    print("Error opening video file")

# Read the first frame of the video
ret, frame = video.read()
height, width = frame.shape[:2]

# Define the 3D coordinates of the calibration object corners
objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2) * square_size

i = 0
j = 0
print("Beginning Data Collection...")
# Loop through each frame of the video
while video.isOpened():
    # Next frame
    ret, img = video.read()
    i = i + 1

    if img is None:
        print("Data Collection Complete.")
        break

    half_width = img.shape[1] // 2
    left_img = img[:, :half_width, :]
    right_img = img[:, half_width:, :]

    # Convert the stereo image pair to grayscale
    left_gray = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
    right_gray = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)

    # Find the corners in the calibration object for each camera
    left_ret, left_corners = cv2.findChessboardCorners(left_gray, board_size, None)
    right_ret, right_corners = cv2.findChessboardCorners(right_gray, board_size, None)

    # If the corners are found in both images, add them to the list of image points
    if left_ret and right_ret:
        j = (j + 1) % 15
        if j == 2:
            left_corners = cv2.cornerSubPix(left_gray, left_corners, (11, 11), (-1, -1), criteria)
            right_corners = cv2.cornerSubPix(right_gray, right_corners, (11, 11), (-1, -1), criteria)
            left_points.append(left_corners)
            right_points.append(right_corners)
            print("SUCCESS on frame #", i)

    if cv2.waitKey(25) & 0xFF == ord('q'):
        break

print("Computing Metrics...")
# Calibrate the stereo camera
retval, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T, E, F = cv2.stereoCalibrate(
    objectPoints=[objp] * len(left_points),
    imagePoints1=left_points,
    imagePoints2=right_points,
    imageSize=left_gray.shape[::-1],
    cameraMatrix1=None,
    distCoeffs1=None,
    cameraMatrix2=None,
    distCoeffs2=None,
    flags=cv2.CALIB_FIX_INTRINSIC
)
print("Complete.")

# Compute the focal length and baseline from the stereo camera parameters
fx = cameraMatrix1[0][0]
baseline = np.linalg.norm(T)

# Print the focal length and baseline
print("Focal length (in pixels):", fx)
print("Baseline (in meters):", baseline)
