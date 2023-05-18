import sys

import numpy as np
import cv2
from picamera2 import Picamera2 
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()


def run():
    frame = picam2.capture_array()

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Threshold of yellow in HSV space
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
  
    # preparing the mask to overlay
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()
    
    params.filterByArea = True
    params.minArea = 500
    params.filterByCircularity = False
    params.filterByConvexity = False
    params.filterByInertia = False

    # Read image
    img = cv2.imread("blob.jpg", cv2.IMREAD_GRAYSCALE)

    detector = cv2.SimpleBlobDetector_create(params)

    # Detect blobs.
    keypoints = detector.detect(img)
      
    result = cv2.bitwise_and(frame, frame, mask = mask)
    to_gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
    
    frame_keypoints = cv2.drawKeypoints(to_gray, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    
    # Show keypoints
    cv2.imshow("Keypoints", frame_keypoints)
    cv2.waitKey(0)


if __name__ == "__main__":
    try:
        run()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        sys.exit()