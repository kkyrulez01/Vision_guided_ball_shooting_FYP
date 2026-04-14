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

    all_rects = []
    all_confidences = []
    bounding_boxes = []
    bounding_boxes_centres = []

    # Templates will contain path of all template images
    for template in templates:
        template_img = cv2.imread(template)
        scaled_template_list = scale_template(template_img) # List of scaled template images
        template_name = Path(template).stem
        for scaled_template in scaled_template_list:
            if scaled_template.shape[0] > frame.shape[0] or scaled_template.shape[1] > frame.shape[1]:
                break # Stop scaling up if template is larger than frame
            else:
                result = cv2.matchTemplate(frame, scaled_template, cv2.TM_CCOEFF_NORMED) # Returns a result matrix with match metric scores of each pixel
                threshold = 0.75 # Adjust this
                loc = np.where(result >= threshold)
                h,w = scaled_template.shape[:-1]
                for pt in zip(*loc[::-1]):
                    all_rects.append([int(pt[0]), int(pt[1]), int(w), int(h)])
                    all_confidences.append(float(result[pt[1], pt[0]]))
            
    # Apply Global NMS to get box with highest score
    indices = apply_NMS(all_rects, all_confidences)
            
    for i in indices:
        x,y,w,h = all_rects[i]
        cv2.rectangle(frame, (x,y), (x + w, y + h), (255,0,255), 4)
        cv2.putText(frame, 'Target', (x-10,y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,0,255), 2)
        
        # Store bounding boxes (x,y,w,h)
        bounding_boxes.append([x,y,w,h])

    return frame, bounding_boxes

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
    
    scales = np.linspace(0.25, 2.75, 6).tolist() # List of scaling ratios
    scaled_template_list = []
    for i in scales:
        #  Resize template image
        scaled_template = cv2.resize(template, None, fx =i, fy=i) # Scale by ratio

        # Store NumPy array 
        scaled_template_list.append(scaled_template)
    
    return scaled_template_list

def apply_NMS(boxes, scores, threshold_conf=0.8, threshold_iou=0.5):
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

def detect_placard_region(frame):
    """
    Run canny edge detection on camera feed to detect location of placard within the image. Returns the 
    pixel coordinates of the 4 corners of placard.
    """
    gray_frame = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2GRAY)
    
    # Apply Gaussian Blur to reduce noise
    blur = cv2.GaussianBlur(gray_frame, (5,5), 0)
    
    # Apply Canny Edge detector
    edges = cv2.Canny(blur, threshold1=10, threshold2=100)
    
    # Apply Hough Line Transform to detect straight edges of the placard
    cdstP = cv2.cvtColor(edges.copy(), cv2.COLOR_GRAY2BGR)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 100, None, 50, 10)
    vertical_lines = []
    angle_tolerance = 20 # 20 degrees angle tolerance

    if lines is not None:
        for i in range(0, len(lines)):
            # Get length of lines
            x1, y1, x2, y2 = lines[i][0]
            length = np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
            
            # Get angle of lines, we only want vertical lines
            angle = abs(np.arctan2(y2-y1, x2-x1) * 180 / np.pi)

            # Only append vertical lines
            if 90 - angle_tolerance <= angle <= 90 + angle_tolerance:
                # Append the line with its length
                vertical_lines.append(([x1, y1, x2, y2], length))

    if not vertical_lines:
        return None
        
    for line in vertical_lines:
        # Draw vertical lines in green
        cv2.line(cdstP, (line[0][0], line[0][1]), (line[0][2], line[0][3]), (0,255,0), 2, cv2.LINE_AA)

    # Return min x, max x
    min_x_line = min(vertical_lines, key=lambda x: x[0][0])
    max_x_line = max(vertical_lines, key=lambda x: x[0][2])
    min_x = min_x_line[0][0]
    max_x = max_x_line[0][2]

    # Return x,y,w,h of region
    h = frame.shape[0]
    w = frame.shape[1]

    # Add padding and num_disparities for right region
    block_padding = 50
    num_disparities = 64

    x_1 = max(0, min_x - num_disparities - block_padding)
    y_1 = 0
    x_2 = min(w, max_x + block_padding)
    y_2 = h

    return x_1, y_1, x_2, y_2, cdstP

def main():
    # Example image
    frame = cv2.imread('src/fyp_wamv_project/fyp_wamv_project/example_images/example_image_1.png')
    placard_ROI = detect_placard_region(frame.copy())
    
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
    template_frame, rects_centres = multi_scale_template_matching(placard_ROI, small_target_templates)
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

def test_detect_placard_region():
    # Example images
    frame_1 = cv2.imread('src/fyp_wamv_project/fyp_wamv_project/example_images/example_image_1.png')
    frame_1_resized = cv2.resize(frame_1, (1280,720))
    frame_2 = cv2.imread('src/fyp_wamv_project/fyp_wamv_project/example_images/left_camera_feed.png')
    frame_3 = cv2.imread('src/fyp_wamv_project/fyp_wamv_project/example_images/right_camera_feed.png')

    region_1_x_1, region_1_y_1, region_1_x_2, region_1_y_2, cdst_1 = detect_placard_region(frame_1_resized)
    region_2_x_1, region_2_y_1, region_2_x_2, region_2_y_2, cdst_2 = detect_placard_region(frame_2)
    region_3_x_1, region_3_y_1, region_3_x_2, region_3_y_2, cdst_3 = detect_placard_region(frame_3)

    frame_1_resized_RGB = cv2.cvtColor(frame_1_resized, cv2.COLOR_BGR2RGB)
    frame_2_RGB = cv2.cvtColor(frame_2, cv2.COLOR_BGR2RGB)
    frame_3_RGB = cv2.cvtColor(frame_3, cv2.COLOR_BGR2RGB)
    # # Get contours
    # contours, hierachy = cv2.findContours(edges, cv2.RETR_EXTERNAL, 
    #                                     cv2.CHAIN_APPROX_SIMPLE)

    # # Draw contours
    # cv2.drawContours(debug_frame, contours, -1, (0,0,255), 1)

    # # Show in a window
    # # Image 1
    # cv2.imshow('Placard region 1', frame_1[region_1_y_1:region_1_y_2, region_1_x_1:region_1_x_2])
    # cv2.imshow('Lines 1', cdst_1)
    # # Image 2
    # cv2.imshow('Placard region 2', frame_2[region_2_y_1:region_2_y_2, region_2_x_1:region_2_x_2])
    # cv2.imshow('Lines 2', cdst_2)
    # # Image 3
    # cv2.imshow('Placard region 3', frame_3[region_3_y_1:region_3_y_2, region_3_x_1:region_3_x_2])
    # cv2.imshow('Lines 3', cdst_3)

    # Plot in matplotlib
    fig, ax = plt.subplots(nrows=3, ncols=2, figsize=(12,18))
    ax[0,0].imshow(frame_1_resized_RGB[region_1_y_1:region_1_y_2, region_1_x_1:region_1_x_2])
    ax[0,0].set_title("Example cropped placard region (1)")

    ax[0,1].imshow(cdst_1)
    ax[0,1].set_title("Example detected lines (1)")
    ax[1,0].imshow(frame_2_RGB[region_2_y_1:region_2_y_2, region_2_x_1:region_2_x_2])
    ax[1,0].set_title("Example cropped placard region (2)")

    ax[1,1].imshow(cdst_2)
    ax[1,1].set_title("Example detected lines (2)")

    ax[2,0].imshow(frame_3_RGB[region_3_y_1:region_3_y_2, region_3_x_1:region_3_x_2])
    ax[2,0].set_title("Example cropped placard region (3)")

    ax[2,1].imshow(cdst_3)
    ax[2,1].set_title("Example detected lines (3)")

    for ax in fig.axes:
        ax.axis('off')

    plt.show()

    cv2.waitKey(0)
    cv2.destroyAllWindows()

def view_scaled_templates():
    # Example template
    template_1 = cv2.imread('src/fyp_wamv_project/fyp_wamv_project/templates/small_target/small_target_1.png')
    scaled_template_list = scale_template(template_1)
    max_w = scaled_template_list[-1].shape[1]
    max_h = scaled_template_list[-1].shape[0]

    # Plot scaled template images
    fig, ax = plt.subplots(nrows=3,ncols=3, figsize=(12,12))
    for i, template in enumerate(scaled_template_list):
        row = i // 3
        col = i % 3
        ax[row, col].set_xlim(0, max_w)
        ax[row, col].set_ylim(max_h, 0)
        ax[row, col].set_xticks([])
        ax[row, col].set_yticks([])
        ax[row, col].imshow(template)
        ax[row, col].set_title(f'Scaled Template {i+1}, Scale: {np.linspace(0.25, 2.75, 6)[i]:.2f}x')

    ax[2,1].imshow(template_1)
    ax[2,1].set_title('Original Template')
    ax[2,1].set_xlim(0, max_w)
    ax[2,1].set_ylim(max_h, 0)
    ax[2,1].set_xticks([])
    ax[2,1].set_yticks([])

    axes_flat = ax.flatten()
    for i in range(len(scaled_template_list), len(axes_flat)):
        if i != 7: # Keep the original template slot visible
            axes_flat[i].set_visible(False)
    plt.show()

if __name__ == "__main__":
    #main()
    #test_detect_placard_region()
    view_scaled_templates()