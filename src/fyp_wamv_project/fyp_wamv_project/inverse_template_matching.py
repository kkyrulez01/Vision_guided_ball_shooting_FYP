import numpy as np
import cv2

class ColorRCEModel():
    def __init__(self, target_hsv, radius):
        self.center = np.array(target_hsv) # Mean HSV value of target (e.g. Color of a buoy at different lightings)
        self.radius = radius # Sphere of influence (Standard deviation)
    
    def predict(self, pixel_hsv):
        # Calculate Euclidean distance between a pixel and the center value
        distance = np.linalg.norm(pixel_hsv - self.center)

        # If inside the sphere, it is a color match (1), else it's background (0)
        return 1 if distance < self.radius else 0

def find_roi_using_RCE(self, frame):
    # Store height and width of the image
    frame_h, frame_w = frame.shape[:2] 

    # Convert frame to  HSV first
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Standard HSV mask to find potential blobs
    lower_bound = np.array([0, 0, 0]) # Black lower bound
    upper_bound = np.array([180, 255, 30])
    mask = cv2.inRange(hsv_frame, lower_bound, higher_bound)

    # Find contours from the mask
    contours, hierachy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best_roi = None
    min_distance = float('inf')

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 500:
            # Get bounding box of candidate 
            x,y,w,h = cv2.boundingRect(cnt)
            candidate_crop = image[y:y+h, x:x+w]

            # Extract its feature vector

def extract_hybrid_feature_vector(roi_image, fixed_size=(32,32)):
    resized_roi = cv2.resize(roi_image, fixed_size)