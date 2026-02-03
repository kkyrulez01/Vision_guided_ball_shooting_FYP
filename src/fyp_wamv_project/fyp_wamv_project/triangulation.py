import cv2
import numpy as np

def calculate_focal_length(image_width_px, HFOV_rad):
    """
    Calculates the focal length of the camera in pixels.

    Arguments:
        image_width_px (int): Width of the image in pixels
        HFOV_rad (float) : Horizontal field-of-view of the camera in terms of radians.

    Returns:
        f_px (float): The focal length of the camera in pixels. 
    """
    f_px = (image_width_px) / (2 * np.tan(HFOV_rad / 2))
    return f_px

def triangulate_points(u, v, disparity_map, frame, baseline, focal_length):
    f = focal_length # in pixels
    B = baseline # in meters

    # Disparity map is a matrix, so to access a pixel at (u,v) we use disparity_map[v, u]
    disparity = disparity_map[v, u] # Dont need to divide by 16 since already done in depth map computation
    Z = (f * B) / disparity # Depth of the point in meters

    if Z > 0: 
        # Draw a circle in the top left of bounding boxes
        cv2.circle(frame, (u, v), 5, (255,0,0), -1)
        # Label the Z value of center point
        cv2.putText(frame, f"Z: {Z:.2f}m", (u + 10, v - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

    return Z

def back_projection(Z, u, v, c_x, c_y, f_x, f_y):
    X = ((u - c_x) * Z / f_x)
    Y = ((v- c_y) * Z / f_y)

    return X,Y
    

