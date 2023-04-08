import av
import numpy as np
import cv2
from stereovision.calibration import StereoCalibrator
from stereovision.exceptions import ChessboardNotFoundError

SQUARE_SIZE_MILLIMETERS = 23

# Define the chessboard pattern size
pattern_size = (8, 6)

# Define the criteria for termination of the iterative algorithms
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2) * SQUARE_SIZE_MILLIMETERS

# Arrays to store object points and image points from all the images
objpoints = []  # 3d points in real world space
imgpoints_l = []  # 2d points in left image plane
imgpoints_r = []  # 2d points in right image plane

print("Starting stream connection...")
# create a PyAV input stream from the raw H264 video stream
container = av.open("udp://192.168.0.218:3000?overrun_nonfatal=1&fifo_size=50000&pkt_size=30000")
stream = container.streams.video[0]
print("Connection established.")


calibrator = StereoCalibrator(8, 6, 2.3, (960, 960))
i = 0
j = 0
print("Beginning Data Collection...")
for packet in container.demux(stream):
    if i > 400:
        break
    j = (j + 1) % 20
    for frame in packet.decode():
        if j == 1:
            i = i + 1

            img = np.array(frame.to_image())

            if img is None:
                print("Data Collection Complete.")
                break

            half_width = img.shape[1] // 2
            img_l = img[:, :half_width, :]
            img_r = img[:, half_width:, :]

            try:
                calibrator._get_corners(img_l)
                calibrator._get_corners(img_r)
            except ChessboardNotFoundError as error:
                print(error)
                print("Pair No ", i, " ignored")
            else:
                calibrator.add_corners((img_l, img_r), True)
                print("SUCCESS on pair No ", i)

print('Starting calibration... It can take several minutes!')
calibration = calibrator.calibrate_cameras()
calibration.export('calib_result')
print('Calibration complete!')
