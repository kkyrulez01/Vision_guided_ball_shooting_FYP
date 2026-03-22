# All the logic should come here
import cv2 
import numpy as np

from fyp_wamv_project.triangulation import calculate_focal_length, triangulation, back_projection
from fyp_wamv_project.depth_map import DepthMap
from fyp_wamv_project.template_matching import *
from fyp_wamv_project.HSV_filter import add_HSV_filter
from fyp_wamv_project.shape_detection import detect_shape

class BaseDetector:
    def __init__(self, left_frame, right_frame):
        self.left_frame = left_frame
        self.right_frame = right_frame

class TemplateMatcher(BaseDetector):
    def __init__(self, left_frame, right_frame, templates:list, baseline):
        super().__init__(left_frame, right_frame)
        self.templates = templates
        self.baseline = baseline
    
    def process_frames(self):
        # Compute depth map
        depth_map = DepthMap(self.left_frame, self.right_frame)
        disparity = depth_map.compute_depth_mapBM() # Matrix of disparity values

        # Multi scale template matching approach
        frame_1 = self.left_frame.copy()

        # Detect placard region
        (x,y,w,h) = detect_placard_region(frame_1)
        placard_ROI = frame_1[y:y+h, x:x+w]

        # Run multi-scale template matching on placard region only
        placard_ROI, bounding_boxes = multi_scale_template_matching(placard_ROI, self.templates)

        # Calculate focal length in pixels
        image_width_px = self.left_frame.shape[1]
        HFOV_rad = 1.3962634 # From wamv_camera.xacro
        focal_length = calculate_focal_length(image_width_px , HFOV_rad) # in pixels

        targets = []
        for box in bounding_boxes:
            x_box,y_box,w_box,h_box = box
            # Calculate actual (x,y) coordinates of bounding boxes on original image
            x_orig = x_box + x
            y_orig = y_box
            w_orig, h_orig = w_box, h_box

            # Get centre coordinates of bounding boxes
            u = x_orig + w_box // 2
            v = y_orig + h_box // 2

            Z = triangulation(u, v, disparity, self.baseline, focal_length)
            # Using Z values, calculate X and Y and draw on frame
            result = back_projection(Z, u, v, 640, 360, focal_length, focal_length, frame_1)
            if result is not None:
                X, Y, Z, frame_1 = result
                targets.append([X,Y,Z])

        return frame_1, targets

class ColorEdgeDetector(BaseDetector):
    def __init__(self, left_frame, right_frame, baseline):
        super().__init__(left_frame, right_frame)
        self.baseline = baseline
    
    def process_frames(self):
        # Compute depth map
        depth_map = DepthMap(self.left_frame, self.right_frame)
        disparity = depth_map.compute_depth_mapBM() # Matrix of disparity values
        
        # HSV Filtering + Shape detection approach
        frame_2 = self.left_frame.copy()

        # Add HSV filter to left frame only since disparity map is aligned with left frame
        # Lower and upper bounds for black
        lower = np.array([0, 0, 0])
        upper = np.array([180, 255, 30])
        _,filtered_frame_2 = add_HSV_filter(frame_2, lower, upper)
        filtered_frame_2 = cv2.normalize(filtered_frame_2, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX) # Normalize for better visualization

        # Detect shapes in the filtered left frame and get their centre coordinates
        filtered_frame_2, shapes_centre = detect_shape(filtered_frame_2)

        # Calculate focal length in pixels
        image_width_px = self.left_frame.shape[1]
        HFOV_rad = 1.3962634 # From wamv_camera.xacro
        focal_length = calculate_focal_length(image_width_px , HFOV_rad) # in pixels

        # Triangulate centre of detected shapes to get Z values
        targets = []
        for centre in shapes_centre:
            u, v = centre
            Z = triangulation(u, v, disparity, self.baseline, focal_length)
            # Using Z values, calculate X and Y and draw on frame
            result = back_projection(Z, u, v, 640, 360, focal_length, focal_length, filtered_frame_2)
            if result is not None:
                X, Y, Z, filtered_frame_2 = result
                targets.append([X,Y,Z])

        # Return disparity map and matched frames
        return disparity, filtered_frame_2, targets

