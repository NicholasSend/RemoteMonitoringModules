import json

import av
import cv2
import numpy as np
from stereovision.calibration import StereoCalibration


def stereo_depth_map(rectified_pair):
    """

    :param rectified_pair:
    :return:
    """
    dmLeft = rectified_pair[0]
    dmRight = rectified_pair[1]
    disparity = sbm.compute(dmLeft, dmRight)
    local_max = disparity.max()
    local_min = disparity.min()
    disparity_grayscale = (disparity - local_min) * (65535.0 / (local_max - local_min))
    disparity_fixtype = cv2.convertScaleAbs(disparity_grayscale, alpha=(255.0 / 65535.0))
    disparity_color = cv2.applyColorMap(disparity_fixtype, cv2.COLORMAP_JET)
    return disparity_color


def calculate_iou(bbox1, bbox2):
    """

    :param bbox1:
    :param bbox2:
    :return:
    """
    x1, y1, w1, h1 = bbox1
    x2, y2, w2, h2 = bbox2
    if x1 >= x2 + w2 or x2 >= x1 + w1 or y1 >= y2 + h2 or y2 >= y1 + h1:
        return 0.0
    else:
        intersect_x1 = max(x1, x2)
        intersect_y1 = max(y1, y2)
        intersect_x2 = min(x1 + w1, x2 + w2)
        intersect_y2 = min(y1 + h1, y2 + h2)
        intersect_area = (intersect_x2 - intersect_x1) * (intersect_y2 - intersect_y1)
        bbox1_area = w1 * h1
        bbox2_area = w2 * h2
        iou = intersect_area / float(bbox1_area)  # + bbox2_area - intersect_area)
        return iou


def load_map_settings(fName):
    """

    :param fName:
    :return:
    """
    global SWS, PFS, PFC, MDS, NOD, TTH, UR, SR, SPWS, loading_settings
    print('Loading parameters from file...')
    f = open(fName, 'r')
    data = json.load(f)
    SWS = data['SADWindowSize']
    PFS = data['preFilterSize']
    PFC = data['preFilterCap']
    MDS = data['minDisparity']
    NOD = data['numberOfDisparities']
    TTH = data['textureThreshold']
    UR = data['uniquenessRatio']
    SR = data['speckleRange']
    SPWS = data['speckleWindowSize']
    # sbm.setSADWindowSize(SWS)
    sbm.setPreFilterType(1)
    sbm.setPreFilterSize(PFS)
    sbm.setPreFilterCap(PFC)
    sbm.setMinDisparity(MDS)
    sbm.setNumDisparities(NOD)
    sbm.setTextureThreshold(TTH)
    sbm.setUniquenessRatio(UR)
    sbm.setSpeckleRange(SR)
    sbm.setSpeckleWindowSize(SPWS)
    f.close()
    print('Parameters loaded from file ' + fName)


# create a PyAV input stream from the raw H264 video stream
container = av.open("udp://192.168.0.218:3001?overrun_nonfatal=1&fifo_size=50000&pkt_size=30000")
stream = container.streams.video[0]

# Depth map default preset
SWS = 5
PFS = 5
PFC = 29
MDS = -30
NOD = 160
TTH = 100
UR = 10
SR = 14
SPWS = 100

# Implementing calibration data
print('Read calibration data and rectifying stereo pair...')
calibration = StereoCalibration(input_folder='calib_result')

# Load the Haar Cascade classifier for stop signs
stop_cascade = cv2.CascadeClassifier('stop_sign.xml')

disparity = np.zeros((640, 1980), np.uint8)
sbm = cv2.StereoBM_create(numDisparities=0, blockSize=21)

load_map_settings("3dmap_set.txt")

# loop over the frames
i = 0
for packet in container.demux(stream):
    i = (i + 1) % 5
    for frame in packet.decode():
        if i == 1:
            # convert the frame to a numpy array
            np_image = np.array(frame.to_image())
            np_image = cv2.cvtColor(np_image, cv2.COLOR_BGR2GRAY)
            half_width = frame.width // 2
            frame_left = np_image[:, :half_width]
            frame_right = np_image[:, half_width:]
            frame_left = cv2.GaussianBlur(cv2.medianBlur(frame_left, 5), (5, 5), 0)
            frame_right = cv2.GaussianBlur(cv2.medianBlur(frame_right, 5), (5, 5), 0)

            # Detect stop signs in the frame
            stop_signs = stop_cascade.detectMultiScale(frame_left, scaleFactor=1.3, minNeighbors=5)

            rectified_pair = calibration.rectify((frame_left, frame_right))
            disparity = stereo_depth_map(rectified_pair)

            # Detect red objects in the frame
            hsv_image = cv2.cvtColor(disparity, cv2.COLOR_BGR2HSV)
            lower_red = np.array([0, 50, 50])
            upper_red = np.array([10, 255, 255])
            mask1 = cv2.inRange(hsv_image, lower_red, upper_red)
            lower_red = np.array([170, 50, 50])
            upper_red = np.array([180, 255, 255])
            mask2 = cv2.inRange(hsv_image, lower_red, upper_red)
            mask = cv2.bitwise_or(mask1, mask2)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            red_detected = False
            red_box_list = list()
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 2500:
                    red_detected = True
                    red_box = cv2.boundingRect(cnt)
                    cv2.rectangle(disparity, (red_box[0], red_box[1]),
                                  (red_box[0] + red_box[2], red_box[1] + red_box[3]), (0, 255, 0), 2)
                    red_box_list.append(red_box)

            if red_detected:
                print('RED DETECTED')
            else:
                print('NO RED DETECTED')

            # Draw a rectangle around each stop sign
            for (x, y, w, h) in stop_signs:
                stop_box = (x, y, w, h)
                cv2.rectangle(disparity, (x, y), (x + w, y + h), (255, 255, 255), 2)
                for red_box in red_box_list:
                    iou = calculate_iou(stop_box, red_box)
                    if iou > 0.4:
                        print("Intersection Detected!")

            # Visualize the point cloud
            cv2.imshow("Point Cloud", disparity)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

# Destroy all the windows
cv2.destroyAllWindows()
