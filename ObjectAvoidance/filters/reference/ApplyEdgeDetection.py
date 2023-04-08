import cv2

# Load the video
video = cv2.VideoCapture("stereopi.mp4")

# Check if the video was successfully loaded
if not video.isOpened():
    print("Error opening video file")

# Read the first frame of the video
ret, frame = video.read()
height, width = frame.shape[:2]
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('output.mp4', fourcc, 20.0, (width, height))

# Loop through each frame of the video
while video.isOpened():
    # Next frame
    ret, frame = video.read()
    if not ret:
        break

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # TODO: ADD SMOOTHING

    # Apply the Canny edge detection algorithm
    edges = cv2.Canny(gray, 50, 150)

    out.write(edges)
    cv2.imshow("Edge Detection", edges)

    if cv2.waitKey(25) & 0xFF == ord('q'):
        break

video.release()
out.release()
cv2.destroyAllWindows()