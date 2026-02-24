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

def triangulation(u, v, disparity_map, baseline, focal_length):
    """
    Calculate Z-value (depth) of a pixel using focal length, baseline and disparity.

    Arguments:
        u, v: Pixel coordinate in disparity map
        disparity_map : Disparity map
        baseline: The baseline or the distance between the two cameras.
        focal_length: Focal length of camera.
    
    Returns:
        The Z value of a pixel.
    """
    f = focal_length # in pixels
    B = baseline # in meters

    # Disparity map is a matrix, so to access a pixel at (u,v) we use disparity_map[v, u]
    disparity = disparity_map[v, u] # Dont need to divide by 16 since already done in depth map computation
    if disparity > 0:
        Z = (f * B) / disparity # Depth of the point in meters

        return Z

def back_projection(Z, u, v, c_x, c_y, f_x, f_y, frame):
    if Z is not None and Z > 0:
    # Calculate X and Y using Z value
        X = ((u - c_x) * Z / f_x)
        Y = ((v- c_y) * Z / f_y)

        # Draw a circle in the centre of bounding boxes
        cv2.circle(frame, (u, v), 5, (255,0,0), -1)
        # Label the Z,X,Y value of centre point
        size, _ = cv2.getTextSize(f"Z: {Z:.2f}m,X: {X:.2f}m,Y: {Y:.2f}m", cv2.FONT_HERSHEY_SIMPLEX, 0.35, 1)
        text_w, text_h = size
        cv2.putText(frame, f"Z: {Z:.2f}m,X: {X:.2f}m,Y: {Y:.2f}m",
                    (u - text_w // 2, v - text_h), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0,255,0), 1)

        return X, Y, Z, frame

    else:
        return None, None, None, frame
    
        
    

