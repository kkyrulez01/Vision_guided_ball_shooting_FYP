import cv2 
import numpy as np
import matplotlib.pyplot as plt

class DepthMap:
    def __init__(self, left_frame, right_frame):
        self.left_frame = cv2.cvtColor(left_frame, cv2.COLOR_BGR2GRAY)
        self.right_frame = cv2.cvtColor(right_frame, cv2.COLOR_BGR2GRAY)

        self.numDisparities_factor = 8 # Adjust this
        self.blockSize = 7 # Must be odd and more than 3 (Default is 21)

        # Create left and right matchers using StereoBM algorithm
        self.left_matcher = cv2.StereoBM_create(numDisparities= 16 * self.numDisparities_factor, blockSize=self.blockSize)
        self.right_matcher = cv2.ximgproc.createRightMatcher(self.left_matcher)

        # Create the WLS filter
        self.wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=self.left_matcher)
        self.wls_filter.setLambda(400) # Smoothing strength
        self.wls_filter.setSigmaColor(1.5) # How much to respect RGB edges

    def compute_depth_mapBM(self):
        # Compute disparity maps
        disp_l = self.left_matcher.compute(self.left_frame, self.right_frame) 
        disp_r = self.right_matcher.compute(self.right_frame, self.left_frame)
        filtered_disp_BM = self.wls_filter.filter(disp_l, self.left_frame, disparity_map_right=disp_r)
        filtered_disp_BM = filtered_disp_BM.astype(np.float32) / 16.0 # Divide by 16 to get actual disparity values

        return filtered_disp_BM

    def compute_depth_mapSGBM(self):
        # Create left and right matchers using StereoSGBM algorithm
        left_matcher = cv2.StereoSGBM_create(
            minDisparity=16, # Minimum disparity value
            numDisparities=16 * 10, # Max disparity - min disparity, must be multiple of 16
            blockSize=5, # Must be an odd number >=1, Usually between 3-11
            P1= 8 * 3 * 5**2, # Controls disparity smoothness, penalty when disparity difference is 1
            P2= 32 * 3 * 5**2, # Controls smoothness, penalty when disparity difference > 1. Rule of thumb: P2 > P1
            disp12MaxDiff=1,
            uniquenessRatio=20,
            speckleWindowSize=200,
            speckleRange=2,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )
        right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)

        # Create the WLS filter
        wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=left_matcher)
        wls_filter.setLambda(400) # Smoothing strength
        wls_filter.setSigmaColor(2) # How much to respect RGB edges

        # Compute disparity map
        disp_l = left_matcher.compute(self.left_frame, self.right_frame).astype(np.float32) / 16.0 # Divide by 16 to get actual disparity values
        disp_r = right_matcher.compute(self.right_frame, self.left_frame).astype(np.float32) / 16.0
        filtered_disp_SGBM = wls_filter.filter(disp_l, self.left_frame, disparity_map_right=disp_r)
        return filtered_disp_SGBM

    def plot_images(self):
        # Display the disparity map
        disparity_sgbm = self.compute_depth_mapSGBM()
        plt.figure()
        plt.subplot(1, 2, 1)
        plt.imshow(self.left_frame, 'gray')
        plt.subplot(1, 2, 2)
        plt.imshow(disparity_sgbm, 'gray')
        plt.show()

# Example usage:
def main():
    import matplotlib.patches as patches

    left_camera = cv2.imread('src/fyp_wamv_project/fyp_wamv_project/example_images/left_camera_feed.png')
    right_camera = cv2.imread('src/fyp_wamv_project/fyp_wamv_project/example_images/right_camera_feed.png')

    left_camera_gray = cv2.cvtColor(left_camera, cv2.COLOR_BGR2GRAY)
    right_camera_gray = cv2.cvtColor(right_camera, cv2.COLOR_BGR2GRAY)

    # With filter
    left_matcher = cv2.StereoBM_create(numDisparities=16 * 8, blockSize=7)
    right_matcher = cv2.ximgproc.createRightMatcher(left_matcher) # Required for WLS filter
    wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=left_matcher)
    wls_filter.setLambda(600)
    wls_filter.setSigmaColor(1.2) 
    disp_l = left_matcher.compute(left_camera_gray, right_camera_gray) 
    disp_r = right_matcher.compute(right_camera_gray, left_camera_gray)
    disparity_1 = wls_filter.filter(disp_l, left_camera, disparity_map_right=disp_r)
    disparity_1 = disparity_1.astype(np.float32) / 16.0 # Divide by 16 to get actual disparity values

    invalid_pixels_count_1 = 0
    for y in range (150, 290):
        for x in range(215, 395):
            if disparity_1[y,x] < 0:
                invalid_pixels_count_1 += 1
    percent_invalid_1 = (invalid_pixels_count_1 / ((395-215) * (290-150))) * 100

    # Without filter
    stereo = cv2.StereoBM_create(numDisparities=16 * 8, blockSize=7)
    disparity_2 = stereo.compute(left_camera_gray, right_camera_gray).astype(np.float32) / 16.0

    invalid_pixels_count_2 = 0
    for y in range (150, 290):
        for x in range(215, 395):
            if disparity_2[y,x] < 0:
                invalid_pixels_count_2 += 1
    percent_invalid_2 = (invalid_pixels_count_2 / ((395-215) * (290-150))) * 100

    # Plot both disparity maps side by side for comparison
    fig, (ax0 ,ax1, ax2) = plt.subplots(3,1, figsize=(10,15))

    # Left camera feed
    ax0.imshow(left_camera_gray, 'gray')
    ax0.set_title("Example left camera feed")
    ax0.axis('off')

    # Without filter
    ax1.imshow(disparity_2, 'gray', vmin=0, vmax=128)
    ax1.set_title(f"Without WLS filter | Percentage of invalid pixels in ROI: {percent_invalid_2:.2f}%")
    ax1.axis('off')

    # With filter
    ax2.imshow(disparity_1, 'gray', vmin=0, vmax=255)
    ax2.set_title(f"With WLS filter | Percentage of invalid pixels in ROI: {percent_invalid_1:.2f}%")
    ax2.axis('off')

    for ax in (ax0, ax1, ax2):
        # Rect
        rect = patches.Rectangle(
        (215,290),
        395-215, # width
        150-290, # height
        edgecolor='red',
        facecolor='none',
    )
        ax.add_patch(rect)

    plt.show() 

    # depth_map.compute_depth_mapBM()

    # depth_map.compute_depth_mapSGBM()
    # depth_map.plot_images()



if __name__ == "__main__":
    main()
    # test_detect_placard_region()
