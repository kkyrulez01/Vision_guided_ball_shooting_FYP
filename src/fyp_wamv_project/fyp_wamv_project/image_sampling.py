import numpy as np
import matplotlib.pyplot as plt
import cv2

def get_grid_divisions(k):
    """
    Given k samples, the function finds all possible (d_v, d_h) where d_v * d_h = k.
    d_v = no. of vertical divisons (i.e. rows)
    d_h = no. of horizontal divisions (i.e. columns)
    """
    divisions = []
    for i in range(1,k+1):
        if k % i == 0: # If it perfectly divides
            d_v = i
            d_h = k // i
            divisions.append((d_v, d_h))
        
    return divisions

def slice_image_to_grid(image, d_v, d_h):
    """
    Slices an image into a dv * dh grid and returns the sub patches.
    """
    img_h, img_w = image.shape[:2]

    # Calculate height and width of each sub-patch
    sub_patch_h = img_h // d_v
    sub_patch_w = img_w // d_h

    patches = []

    # Loop through grid and extract each patch
    for row in range(d_v):
        for column in range(d_h):
            # Calculate pixel coordinates of the slice
            y_start = row * sub_patch_h
            x_start = column * sub_patch_w

            # If image size doesn't divide perfectly
            y_end = (row + 1) * sub_patch_h if row < d_v - 1 else img_h
            x_end = (column + 1) * sub_patch_w if column < d_h - 1 else img_w

            patch = image[y_start:y_end, x_start:x_end]
            patches.append(patch)
    
    return patches

def generate_S_k_set(roi_image, max_k):
    """
    Generate the image sample set S_k.
    """
    S = {}

    for k in range(1, max_k + 1):
        S[k] = []

        # Get all grid combinations for this k (e.g. for k=4: 1x4, 2x2, 4x1)
        grid_combinations = get_grid_divisions(k)

        for (d_v, d_h) in grid_combinations:
            # Slice image according to this specific grid
            patches = slice_image_to_grid(roi_image, d_v, d_h)

            # Store it in S_k set
            S[k].append({
                'Grid shape': f"{d_v} x {d_h}",
                'patches': patches,
            })

    return S

def main():
    # Example usage
    image = cv2.imread('src/fyp_wamv_project/fyp_wamv_project/example_images/left_camera_feed.png')

    # Uncomment to see result after slicing image to grids
    k = 24 # Adjust this
    combinations = get_grid_divisions(k)
    for (d_v, d_h) in combinations:
        patches = slice_image_to_grid(image, d_v, d_h)

        fig, axes = plt.subplots(d_v, d_h, figsize=(d_h * 3, d_v * 3))

        if k == 1:
            axes = np.array([[axes]])
        elif d_v == 1: # Just 1 vertical cut
            axes = axes[np.newaxis, :]
        elif d_h == 1: # Just 1 horizontal cut
            axes = axes[:, np.newaxis]

        patch_idx = 0
        for row in range(d_v):
            for column in range(d_h):
                ax = axes[row, column]
                # Convert BGR to RGB
                rgb_patch = cv2.cvtColor(patches[patch_idx], cv2.COLOR_BGR2RGB)

                ax.imshow(rgb_patch)
                ax.axis('off')

                patch_idx += 1
        
        # Add spacing between the subplots to visualise the slices
        plt.subplots_adjust(wspace=0.1, hspace=0.1)
        plt.show()

    # Uncomment to verify S_k set
    sample_sets = generate_S_k_set(image, max_k = 10)
    for k, combinations in sample_sets.items():
        print(f"S_{k} contains {len(combinations)} grid configurations:")
        for config in combinations:
            print(f"  - Grid {config['Grid shape']} -> Yields {len(config['patches'])} sub-patches")
            print("-" * 30)

if __name__ == 'main':
    main()