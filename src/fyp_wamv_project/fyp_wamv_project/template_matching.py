import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
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

    # Only search the top 2/3 of the image
    y_start, x_start = 0, 0
    y_end = int((2/3) * frame.shape[0])
    x_end = frame.shape[1]
    search_area = frame[y_start:y_end, x_start:x_end]

    all_rects = []
    all_confidences = []
    rects_centres = []

    # Templates will contain path of all template images
    for template in templates:
        template_img = cv2.imread(template)
        scaled_template_list = scale_template(template_img) # List of scaled template images
        template_name = Path(template).stem
        for scaled_template in scaled_template_list:
            result = cv2.matchTemplate(search_area, scaled_template, cv2.TM_CCOEFF_NORMED) # Returns a result matrix with match metric scores of each pixel
            threshold = 0.75 # Adjust this
            loc = np.where(result >= threshold)
            h,w = scaled_template.shape[:-1]
            for pt in zip(*loc[::-1]):
                all_rects.append([int(pt[0]), int(pt[1] + y_start), int(w), int(h)])
                all_confidences.append(float(result[pt[1], pt[0]]))
            
    # Apply Global NMS to get box with highest score
    indices = apply_NMS(all_rects, all_confidences)
            
    for i in indices:
        x,y,w,h = all_rects[i]
        cv2.rectangle(frame, (x,y), (x + w, y + h), (0,255,0), 4)
        cv2.putText(frame, 'Target', (x-10,y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
        rects_centres.append((x+w//2, y+h//2))
        # Get template name from path
        # template_name = Path(template).stem
        # templates_locations[template_name] = (top_left, bottom_right)

    return frame, rects_centres

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
    
    scales = np.arange(0.25, 3, 0.25).tolist() # List of scaling ratios
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
    indices = np.array(indices).flatten().tolist()

    return indices

def stamp_template(ax, template, x, y, scale, alpha=0.8):
    """
    Stamps the template onto the axes at a specific scale.

    Arguments:
        ax: The main Axis (where the image is plotted)
        template: The original template image array
        x, y: The top-left coordinates of the match
        scale: The scale factor
    """

    imagebox = OffsetImage(template, zoom=scale, alpha=alpha)
    h, w = template.shape[:2]
    cx = x + (w * scale) / 2
    cy = y + (h * scale) / 2

    ab = AnnotationBbox(imagebox, (cx, cy), frameon=False)
    ax.add_artist(ab)


def main():
    # Example image
    frame = cv2.imread('src/fyp_wamv_project/fyp_wamv_project/example_images/example_image_1.png')
    template_frame = frame.copy()

    # Path to templates
    templates_dir = Path('src/fyp_wamv_project/fyp_wamv_project/templates')
    small_target_template_dir = templates_dir / 'small_target'
    small_target_templates = []
    for item in small_target_template_dir.iterdir():
        small_target_templates.append(str(item))
                
    # # Test scale_template
    # template_1 = cv2.imread(templates[0])
    # scaled_template_list = scale_template(template_1)
    
    # # Plot scaled template images
    # nrows = len(scaled_template_list) // 3 + 1
    # ncols = 3
    # fig, ax = plt.subplots(nrows,ncols, figsize=(15,8))
    # ax = ax.flatten()
    
    # for i in range(len(scaled_template_list)):
    #     ax[i].imshow(scaled_template_list[i])
    #     ax[i].set_xlim(0,500)
    #     ax[i].set_ylim(500,0)

    # for j in range(len(scaled_template_list), len(ax)):
    #     ax[j].axis('off')
    
    # plt.show()

    # Test multi_scale_template_matching
    template_frame, rects_centres = multi_scale_template_matching(template_frame, small_target_templates)
    print(rects_centres)

    # # Copy this frame to stack scaled templates on the centre of bounding boxes
    # frame_stacking = frame.copy()

    fig, ax = plt.subplots(nrows=1,ncols=2,figsize=(12,8))
    ax[0].imshow(frame) # Original image
    ax[1].imshow(template_frame) # Image with template matching done

    # ax[1].imshow(frame) # Stacked templates image
    # # For stacking images
    # for i in range(len(rects_indices)):
    #     x,y,w,h = rects_indices[i]
    #     for j in range(len(scaled_template_list)):
    #         left, right, bottom, top = x, x + scaled_template_list[j].shape[0], y, y + scaled_template_list[j].shape[1] 
    #         ax[1].imshow(scaled_template_list[0], extent=[left,right,bottom,top], alpha=0.1)
    # ax[0].set_xlim(0,900)
    # ax[0].set_ylim(720,0)
    # ax[1].set_xlim(0,900)
    # ax[1].set_ylim(720,0)

    ax[0].axis('off'), ax[1].axis('off')
    plt.show()

if __name__ == "__main__":
    main()





        