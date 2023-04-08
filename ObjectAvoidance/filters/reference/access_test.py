import av
import cv2
import numpy as np

# create a PyAV input stream from the raw H264 video stream
container = av.open("udp://192.168.0.218:3000?overrun_nonfatal=1&fifo_size=50000&pkt_size=30000")
stream = container.streams.video[0]

stereo = cv2.StereoSGBM_create(
    numDisparities=64,
    blockSize=3
)

baseline = 0.1  # in meters
focal_length = 600  # in pixels

# Load the Haar Cascade classifier for stop signs
stop_cascade = cv2.CascadeClassifier('stop_sign.xml')

# loop over the frames
i = 0
for packet in container.demux(stream):
    i = (i + 1) % 10
    for frame in packet.decode():
        if i == 1:
            # convert the frame to a numpy array
            np_image = np.array(frame.to_image())

            gray = cv2.cvtColor(np_image, cv2.COLOR_BGR2GRAY)

            half_width = frame.width // 2
            left_img = gray[:, :half_width]
            right_img = gray[:, half_width:]

            left_img = cv2.GaussianBlur(left_img, (5, 5), 0)
            right_img = cv2.GaussianBlur(right_img, (5, 5), 0)

            # Detect stop signs in the frame
            stop_signs = stop_cascade.detectMultiScale(gray, scaleFactor=1.3, minNeighbors=5)

            # Compute the depth map using the StereoSGBM object
            disp = stereo.compute(left_img, right_img)
            disp = cv2.medianBlur(disp, 5)

            # Convert the disparity map to a depth map
            depth = np.zeros(disp.shape, dtype=np.float32)
            mask = disp > 0
            depth[mask] = baseline * 10 / disp[mask]

            # Normalize the depth map to values between 0 and 1
            depth_normalized = cv2.normalize(depth, None, 0, 1, cv2.NORM_MINMAX)

            # Draw a rectangle around each stop sign
            for (x, y, w, h) in stop_signs:
                cv2.rectangle(depth_normalized, (x, y), (x + w, y + h), (255, 255, 255), 2)

            # display the frame
            cv2.imshow("Frame", depth_normalized)

            # wait for a key press
            key = cv2.waitKey(1) & 0xFF

            # if the 'q' key was pressed, break from the loop
            if key == ord("q"):
                break

# release the resources
cv2.destroyAllWindows()
