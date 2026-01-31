import cv2
import numpy as np

def calculate_focal_length(image_width_px, HFOV_rad):
    f_px = (image_width_px) / (2 * np.tan(HFOV_rad / 2))
    return f_px
    
def triangulate_points(u, v, disparity_map, frame, baseline, focal_length):
    f = focal_length # in pixels
    B = baseline # in meters

    # Disparity map is a matrix, so to access a pixel at (u,v) we use disparity_map[v, u]
    disparity = disparity_map[v, u] # Dont need to divide by 16 since already done in depth map computation
    Z = (f * B) / disparity # Depth of the point in meters

    # Draw a circle in the center of bounding boxes
    cv2.circle(frame, (u, v), 5, (255,0,0), -1)
    # Label the Z value of center point
    cv2.putText(frame, f"Z: {Z:.2f}m", (u + 10, v - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
