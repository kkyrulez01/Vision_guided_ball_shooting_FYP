import cv2
import numpy as np

def add_HSV_filter(frame, lower, upper):
    # Convert BGR to HSV
    frame_hsv = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2HSV) # Copy the frame then perform color space conversion

    # Create a mask using defined bounds
    mask = cv2.inRange(frame_hsv, lower, upper)

    # Apply the mask to get the filtered image
    filtered_frame_HSV = cv2.bitwise_and(frame, frame, mask=mask)
    filtered_frame_BGR = cv2.cvtColor(filtered_frame_HSV, cv2.COLOR_HSV2BGR)
    cv2.normalize(filtered_frame_BGR, filtered_frame_BGR, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX) # Normalize for better visualization

    # # Display the results
    # cv2.imshow("Original frame", frame)
    # cv2.imshow("HSV frame", frame_hsv)
    # cv2.imshow("Mask", mask)
    # cv2.imshow("Filtered frame", filtered_frame_BGR)
    # cv2.waitKey(0)

    return filtered_frame_BGR
    
def main():
    # Load example image
    image = cv2.imread('/home/kky/fyp_ws/src/fyp_wamv_project/fyp_wamv_project/templates/small_target_template.png')
    # Define lower and upper bound for black color
    lower = np.array([0, 0, 0]) 
    upper = np.array([180, 255, 30])
    add_HSV_filter(image, lower, upper)

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()