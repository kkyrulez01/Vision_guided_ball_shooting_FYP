import cv2
import numpy as np
import matplotlib.pyplot as plt
from fyp_wamv_project.HSV_filter import add_HSV_filter

def detect_shape(frame):
    gray_frame = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2GRAY) # Convert to grayscale first

    # # Trackbars to adjust Canny edge detection minVal, maxVal
    # cv2.namedWindow('Parameters')
    # cv2.createTrackbar('minVal', 'Parameters', 0, 500, lambda x: None)
    # cv2.createTrackbar('maxVal', 'Parameters', 0, 500, lambda x: None)

    # # Trackbar to adjust contour area threshold
    # cv2.createTrackbar('Area', 'Parameters', 0, 1000, lambda x: None)

    # Use Canny edge detection, returns a binary image with thin edges
    # minVal = cv2.getTrackbarPos('minVal', 'Parameters')
    # maxVal = cv2.getTrackbarPos('maxVal', 'Parameters')
    edges = cv2.Canny(gray_frame, 50, 150) # Input image, minVal, maxVal

    # Get contours
    contours, hierachy = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, 
                                          cv2.CHAIN_APPROX_SIMPLE) # Source image, contour retrieval mode, contour approximation method

    # Only draw contours that are large enough (avoid noise)
    contours_frame = frame.copy()
    # Store the centre coordinates and top left of the shape
    shapes_top_left = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        # areaThreshold = cv2.getTrackbarPos('Area', 'Parameters')
        if area > 500:
            cv2.drawContours(contours_frame, cnt, -1, (0,255,0), 3) # -1 to draw all contours, color, thickness
            peri = cv2.arcLength(cnt, True) # True indicates that the contour is closed
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True) 
            x, y, w, h = cv2.boundingRect(approx)
            cv2.rectangle(contours_frame, (x,y), (x+w, y+h), (255,0,255), 2)
            shapes_top_left.append((x,y))


    return contours_frame,shapes_top_left

    # # Plot image
    # plt.subplot(1,3,1)
    # plt.imshow(frame, cmap='gray') # Show original image
    # plt.subplot(1,3,2)
    # plt.imshow(edges, cmap='gray') # Show edges detected
    # plt.subplot(1,3,3)
    # plt.imshow(contours_frame)
    # plt.show()

def main():
    frame = cv2.imread('src/fyp_wamv_project/fyp_wamv_project/example_images/left_camera_feed.png')
    lower = np.array([0, 0, 0]) 
    upper = np.array([180, 255, 30])
    frame_HSV = add_HSV_filter(frame, lower,upper)

    detect_shape(frame_HSV)

if __name__=="__main__":
    main()

