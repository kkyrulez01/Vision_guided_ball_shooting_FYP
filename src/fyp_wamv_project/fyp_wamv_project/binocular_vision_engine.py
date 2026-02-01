# All the logic should come here
import cv2 
import numpy as np

from fyp_wamv_project.depth_map import DepthMap
from fyp_wamv_project.template_matching import template_matching, get_template_centre
from fyp_wamv_project.HSV_filter import add_HSV_filter
from fyp_wamv_project.triangulation import calculate_focal_length, triangulate_points

class StereoProcessor:
    def __init__(self, left_frame, right_frame, baseline, templates: list):
        # Takes in left and right frames along with list of templates
        self.left_frame = left_frame
        self.right_frame = right_frame
        self.baseline = baseline # in meters

        self.templates = templates
        
        
    def process_frames(self):
        # Compute depth map
        depth_map = DepthMap(self.left_frame, self.right_frame)
        disparity = depth_map.compute_depth_mapBM() # Matrix of disparity values
        
        # Add HSV filter to left frame only since disparity map is aligned with left frame
        # For black color (since target is black)
        lower = np.array([0, 0, 0])
        upper = np.array([180, 255, 30])
        filtered_left_frame = add_HSV_filter(self.left_frame, lower, upper)
        filtered_left_frame_norm = cv2.normalize(filtered_left_frame, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX) # Normalize for better visualization
        # # Perform template matching on the filtered left frame
        left_frame_matched, templates_locations = template_matching(filtered_left_frame_norm, self.templates)
        templates_centres = get_template_centre(templates_locations) # Get centre coordinates of matched templates

        # Calculate focal length in pixels
        image_width_px = self.left_frame.shape[1]
        HFOV_rad = 1.3962634 # From wamv_camera.xacro
        focal_length = calculate_focal_length(image_width_px , HFOV_rad) # in pixels

        # Calculate depth of centre point of each bounding box and 
        for template in templates_centres:
            centre_u, centre_v = templates_centres[template]
            triangulate_points(centre_u, centre_v, disparity, left_frame_matched, self.baseline, focal_length)

        # Return disparity map and matched frames
        return disparity, left_frame_matched


