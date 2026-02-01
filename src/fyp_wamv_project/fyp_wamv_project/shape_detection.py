import cv2
import numpy as np
import matplotlib.pyplot as plt
from fyp_wamv_project.HSV_filter import add_HSV_filter

def detect_shape(frame):
    gray_frame = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2GRAY) # Convert to grayscale first

    # Use Canny edge detection, returns a binary image with thin edges
    edges = cv2.Canny(gray_frame, 50, 150) # Input image, minVal, maxVal

    # Plot image
    plt.subplot(1,2,1)
    plt.imshow(frame, cmap='gray') # Show original image
    plt.subplot(1,2,2)
    plt.imshow(edges, cmap='gray') # Show edges detected

    plt.show()

def main():
    frame = cv2.imread('/home/kky/Pictures/left_camera_feed.png')
    lower = np.array([0, 0, 0]) 
    upper = np.array([180, 255, 30])
    frame_HSV = add_HSV_filter(frame, lower,upper)

    detect_shape(frame_HSV)

if __name__=="__main__":
    main()

