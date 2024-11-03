import urllib.request
import numpy as np
import cv2
import time

url = "http://192.168.0.207/cam-hi.jpg"
cv2.namedWindow("ESP32 Feed", cv2.WINDOW_AUTOSIZE)

def is_blue_detected(frame, blue_pixel_threshold=0.20):
    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define range of blue color in HSV
    lower_blue = np.array([75, 75, 23])
    upper_blue = np.array([110, 255, 230])

    
    hsv_blurred = cv2.GaussianBlur(hsv, (11, 11), 0)

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv_blurred, lower_blue, upper_blue)

    # Apply morphological operations to remove noise
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((20, 20), np.uint8))


    # Removes small components that is mainly noise
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    for i in range(1, num_labels):
        if stats[i, cv2.CC_STAT_AREA] < 500:  # Adjust the area threshold as needed
            mask[labels == i] = 0

    blue_pixel_count = np.sum(mask > 0)
    total_pixel_count = mask.size
    blue_pixel_percentage = blue_pixel_count / total_pixel_count

    cv2.imshow("Masked Feed", mask)

    return blue_pixel_percentage >= blue_pixel_threshold

    # contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # # Check if any contour is large enough
    # for contour in contours:
    #     area = cv2.contourArea(contour)
    #     if area > 1000:  # Adjust this threshold as needed
    #         print("Blue")
    #         return True
    #     else:
    #         print("None")
    #         return False

    
# Track frames for 5 seconds
start_time = time.time()
detection_count = 0
frame_count = 0

while True:
    imgFromURL = urllib.request.urlopen(url)
    npImg = np.array(bytearray(imgFromURL.read()), dtype=np.uint8)
    frame = cv2.imdecode(npImg, -1)

    # Check if the current frame has significant blue detection
    if is_blue_detected(frame):
        print("Blue")
        detection_count += 1
    else:
        print("None")

    frame_count += 1

    cv2.imshow("ESP32 Feed", frame)

    # If 5 seconds have passed, check the blue detection percentage
    if time.time() - start_time >= 5:
        # Calculate the detection rate over the past 5 seconds
        detection_rate = detection_count / frame_count
        if detection_rate >= 0.60:
            print("Blue detected in majority of frames over 5 seconds!")
        else:
            print("No significant blue detection over the past 5 seconds.")
        
        # Reset for the next 5 seconds
        detection_count = 0
        frame_count = 0
        start_time = time.time()

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()