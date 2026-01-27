import cv2
import numpy as np
from pathlib import Path

def template_matching(frame, templates: list):
    """
    Perform template matching on given frame using provided templates.

    Args:
        frame (cv2.Mat): The frame to perform template matching on.
        templates (list): List of file paths to template images.
    
    Returns:
        Boxes drawn on frame where templates are matched.
        Dictionary of template locations.
    """
    # Dictionary to store coordinates of matched templates
    template_locations = {}
    # Templates will contain path of all template images
    for template in templates:
        template_img = cv2.imread(template)
        # template_img_rgb = cv2.cvtColor(template_img, cv2.COLOR_BGR2RGB) 
        result = cv2.matchTemplate(frame, template_img, cv2.TM_CCOEFF) # Returns a result matrix with match metric scores of each pixel

        # Find min and max values and locations in the result matrix
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result) 

        # Locate top left and bottom right points of bounding box
        top_left = max_loc
        h, w = template_img.shape[:-1] # Height and width of template image
        bottom_right = (top_left[0] + w, top_left[1] + h)

        # Draw red bounding box around matched region
        cv2.rectangle(frame, top_left, bottom_right, (0,255,0), 2)
        cv2.putText(frame, 'Target', (top_left), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0,255,0), 1)

        # Get template name from path
        template_name = Path(template).stem
        templates_locations[template_name] = (top_left, bottom_right)

    return frame, templates_locations

def get_template_centre(templates_locations: dict):
    # Dictionary to store centre coordinates of matched templates
    template_centres = {}
    for template in templates_locations:
        top_left, bottom_right = templates_locations[template]
        w = bottom_right[0] - top_left[0]
        h = bottom_right[1] - top_left[1]

        # Calculate center coordinates of template (u,v)
        centre_u = top_left[0] + w // 2
        centre_v = top_left[1] + h // 2

        template_centres[template] = (centre_u, centre_v)
    
    return template_centres
        