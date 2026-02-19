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
    def __init__(self, left_frame, right_frame, templates:list):
        super().__init__(left_frame, right_frame)
        self.templates = templates
    
    def process_frames(self):
        # Compute depth map
        depth_map = DepthMap(self.left_frame, self.right_frame)
        disparity = depth_map.compute_depth_mapBM() # Matrix of disparity values

        # Multi scale template matching approach
        frame_1 = self.left_frame.copy()
        frame_1, rects_indices, scaled_template_list = multi_scale_template_matching(frame_1, self.templates)

        return disparity, frame_1

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
        filtered_frame_2 = add_HSV_filter(frame_2, lower, upper)
        filtered_frame_2 = cv2.normalize(filtered_frame_2, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX) # Normalize for better visualization

        # Detect shapes in the filtered left frame and get their centre coordinates
        filtered_frame_2, shapes_centre = detect_shape(filtered_frame_2)

        # Calculate focal length in pixels
        image_width_px = self.left_frame.shape[1]
        HFOV_rad = 1.3962634 # From wamv_camera.xacro
        focal_length = calculate_focal_length(image_width_px , HFOV_rad) # in pixels

        # Triangulate centre of detected shapes to get Z values
        for centre in shapes_centre:
            u, v = centre
            Z = triangulation(u, v, disparity, self.baseline, focal_length)
            # Using Z values, calculate X and Y and draw on frame
            filtered_frame_2 = back_projection(Z, u, v, 640, 360, focal_length, focal_length, filtered_frame_2)

        # Return disparity map and matched frames
        return disparity, filtered_frame_2


