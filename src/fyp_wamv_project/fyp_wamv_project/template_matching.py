import cv2
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

def multi_scale_template_matching(frame, templates: list):
    """
    Perform multi scale template matching on given frame using provided templates. 

    Args:
        frame (cv2.Mat): The frame to perform template matching on.
        templates (list): List of file paths to template images.
    
    Returns:
        Boxes drawn on frame where templates are matched.
        Dictionary of template locations.
    """
    # Dictionary to store coordinates of matched templates
    templates_locations = {}
    # Templates will contain path of all template images
    for template in templates:
        template_img = cv2.imread(template)
        scaled_template_list = scale_template(template_img) # List of scaled template images
        template_name = Path(template).stem

        rects = []
        confidences = []
        for scaled_template in scaled_template_list:
            result = cv2.matchTemplate(frame, scaled_template, cv2.TM_CCOEFF_NORMED) # Returns a result matrix with match metric scores of each pixel
            threshold = 0.75 # Adjust this
            loc = np.where(result >= threshold)
            h,w = scaled_template.shape[:-1]
            for pt in zip(*loc[::-1]):
                rects.append([int(pt[0]), int(pt[1]), int(w), int(h)])
                confidences.append(float(result[pt[1], pt[0]]))
            
            # Apply NMS to get box with highest score
            indices = apply_NMS(rects, confidences)
            
            for i in indices:
                x,y,w,h = rects[i]
                cv2.rectangle(frame, (x,y), (x + w, y + h), (0,255,0), 2)
                cv2.putText(frame, f'{template_name}', (x-10,y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
                
            # Get template name from path
            # template_name = Path(template).stem
            # templates_locations[template_name] = (top_left, bottom_right)

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

def scale_template(template):
    """
    Takes in a template image (numpy.ndarray) and resize it through a range of scales.

    Returns a list of resized images (numpy.ndarray).
    """
    original_height = template.shape[0]
    original_width = template.shape[1]
    
    scales = np.arange(0.25, 2, 0.25).tolist() # List of scaling ratios
    scaled_template_list = []
    for i in scales:
        #  Resize template image
        scaled_template = cv2.resize(template, None, fx =i, fy=i) # Scale by ratio

        # Store NumPy array 
        scaled_template_list.append(scaled_template)
    
    return scaled_template_list

def apply_NMS(boxes, scores, threshold_conf=0.75, threshold_iou=0.5):
    """
    Performs Non-Maximum Suppression to filter out overlapping bounding boxes, keeping only the one
    with the highest confidence score.

    Arguments:
        boxes: List of [x, y, w, h]
        scores: List of confidence scores from matchTemplate
        threshold_conf: Threshold confidence score
        threshold_iou: Overlap threshold (e.g. 0.5 means 50% overlap)
    
    Returns:
        Bounding box [x, y, w, h] with the highest confidence score.
    """
    boxes = np.array(boxes).tolist()
    scores = np.array(scores).tolist()

    # Indices will contain the ID of the boxes to keep
    indices = cv2.dnn.NMSBoxes(boxes, scores, threshold_conf, threshold_iou)

    return indices

def main():
    frame = cv2.imread('/home/kky/Pictures/left_camera_feed.png')

    # Templates
    templates = ['/home/kky/fyp_ws/src/fyp_wamv_project/fyp_wamv_project/templates/small_target_template.png',
                '/home/kky/fyp_ws/src/fyp_wamv_project/fyp_wamv_project/templates/big_target_template.png']

    # # Test scale_template
    # template_1 = cv2.imread(templates[0])
    # scaled_template_list = scale_template(template_1)
    
    # # Plot scaled template images
    # nrows = len(scaled_template_list) // 3 + 1
    # ncols = 3
    # fig, ax = plt.subplots(nrows,ncols, figsize=(12,10))
    # ax = ax.flatten()
    
    # for i in range(len(scaled_template_list)):
    #     ax[i].imshow(scaled_template_list[i])

    # for j in range(len(scaled_template_list), len(ax)):
    #     ax[j].axis('off')
    
    # plt.show()

    # Test multi_scale_template_matching
    frame, templates_locations = multi_scale_template_matching(frame, templates)
    plt.imshow(frame)
    plt.show()
if "__name__" == "__main__":
    main()





        