# All the logic should come here
import cv2 
import numpy as np

from fyp_wamv_project.triangulation import *
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
        # Detect placard region
        result = detect_placard_region(self.left_frame)

        if result is None:
            return self.left_frame
        else:
            x_1, y_1, x_2, y_2 = result
            left_placard_ROI = self.left_frame[y_1:y_2, x_1:x_2]
            right_placard_ROI = self.right_frame[y_1:y_2, x_1:x_2]

            # Compute depth map
            depth_map = DepthMap(left_placard_ROI, right_placard_ROI)
            disparity = depth_map.compute_depth_mapBM() # Matrix of disparity values

            # Multi scale template matching approach
            frame_1 = self.left_frame.copy()

            # Run multi-scale template matching on placard region only
            left_placard_ROI, bounding_boxes = multi_scale_template_matching(left_placard_ROI, self.templates)

            # Calculate focal length in pixels
            image_width_px = self.left_frame.shape[1]
            HFOV_rad = 1.3962634 # From wamv_camera.xacro
            focal_length = calculate_focal_length(image_width_px , HFOV_rad) # in pixels

            targets = []
            for box in bounding_boxes:
                # Get bounding box centres within the cropped out local disparity map
                x_box,y_box,w_box,h_box = box
                u_local = x_box + w_box // 2
                v_local = y_box + h_box // 2
                # Perform triangulation to get depth of those bounding boxes within local disparity map
                Z = triangulation(u_local, v_local, disparity, self.baseline, focal_length)
                # Using Z values, calculate X and Y on original frame
                # Calculate actual (x,y) coordinates of bounding boxes on original frame
                x_orig = x_box + x_1
                y_orig = y_box
                w_orig, h_orig = w_box, h_box

                # Get centre coordinates of bounding boxes on original frame
                u_orig = x_orig + w_box // 2
                v_orig = y_orig + h_box // 2
                result = back_projection_no_drawing(Z, u_orig, v_orig, 640, 360, focal_length, focal_length)
                if result is not None:
                    if Z is None:
                        continue
                    else:
                        X, Y, Z, = result
                        # Append to targets
                        targets.append([X,Y,Z])

                        # Now draw on original frame
                        # Draw a circle in the centre of bounding boxes
                        cv2.circle(frame_1, (u_orig, v_orig), 5, (255,0,0), -1)

                        # Label the Z,X,Y value of centre point
                        size, _ = cv2.getTextSize(f"Z: {Z:.2f}m,X: {X:.2f}m,Y: {Y:.2f}m", cv2.FONT_HERSHEY_SIMPLEX, 0.35, 1)
                        text_w, text_h = size
                        cv2.putText(frame_1, f"Z: {Z:.2f}m,X: {X:.2f}m,Y: {Y:.2f}m",
                                    (u_orig - text_w // 2, v_orig - text_h), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0,255,0), 1)

                        # Label name of target
                        cv2.rectangle(frame_1, (x_orig,y_orig), (x_orig + w_orig, y_orig + h_orig), (255,0,255), 4)
                        cv2.putText(frame_1, 'Target', (x_orig-10,y_orig-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,0,255), 2)

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

