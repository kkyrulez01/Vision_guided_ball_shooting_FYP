import cv2
import numpy as np

def add_HSV_filter(frame):
    # Convert BGR to HSV
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define lower and upper bounds for black color in HSV
    lower = np.array([0, 0, 0])
    upper = np.array([180, 255, 30])

    # Create a mask using defined bounds
    mask = cv2.inRange(frame_hsv, lower, upper)

    cv2.imshow("Original frame", frame)
    cv2.imshow("HSV frame", frame_hsv)
    cv2.imshow("Mask", mask)

    cv2.waitKey(0)

def main():
    # Load example image
    image = cv2.imread('/home/kky/Pictures/Screenshots/test_image.png')
    add_HSV_filter(image)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()