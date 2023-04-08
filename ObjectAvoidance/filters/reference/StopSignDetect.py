import av
import numpy as np
import cv2

# create a PyAV input stream from the raw H264 video stream
container = av.open("udp://192.168.0.218:3000?overrun_nonfatal=1&fifo_size=50000&pkt_size=30000")
stream = container.streams.video[0]

# Load the Haar Cascade classifier for stop signs
stop_cascade = cv2.CascadeClassifier('stop_sign.xml')

# loop over the frames
i = 0
for packet in container.demux(stream):
    i = (i + 1) % 5
    for frame in packet.decode():
        if i == 1:
            # convert the frame to a numpy array
            np_image = np.array(frame.to_image())

            # Convert the frame to grayscale
            gray = cv2.cvtColor(np_image, cv2.COLOR_BGR2GRAY)

            half_width = frame.width // 2
            left_img = gray[:, :half_width]

            # Detect stop signs in the frame
            stop_signs = stop_cascade.detectMultiScale(left_img, scaleFactor=1.3, minNeighbors=5)

            # Draw a rectangle around each stop sign
            for (x, y, w, h) in stop_signs:
                cv2.rectangle(left_img, (x, y), (x + w, y + h), (255, 255, 255), 2)

            # Display the frame with the stop sign rectangles
            cv2.imshow('Stop Sign Detector', left_img)

            # Exit the loop if the 'q' key is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

# Close all windows
cv2.destroyAllWindows()
