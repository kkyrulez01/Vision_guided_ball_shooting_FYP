import cv2 
import numpy as np
import matplotlib.pyplot as plt

class DepthMap:
    def __init__(self, left_frame, right_frame):
        self.left_frame = cv2.cvtColor(left_frame, cv2.COLOR_BGR2GRAY)
        self.right_frame = cv2.cvtColor(right_frame, cv2.COLOR_BGR2GRAY)
    
    def compute_depth_mapBM(self):
        numDisparities_factor = 8 # Adjust this
        blockSize = 7 # Must be odd and more than 3 (Default is 21)

        # Create left and right matchers using StereoBM algorithm
        left_matcher = cv2.StereoBM_create(numDisparities= 16 * numDisparities_factor, blockSize=blockSize)
        right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)

        # Create the WLS filter
        wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=left_matcher)
        wls_filter.setLambda(400) # Smoothing strength
        wls_filter.setSigmaColor(1.5) # How much to respect RGB edges

        # Compute disparity maps
        disp_l = left_matcher.compute(self.left_frame, self.right_frame) 
        disp_r = right_matcher.compute(self.right_frame, self.left_frame)
        filtered_disp_BM = wls_filter.filter(disp_l, self.left_frame, disparity_map_right=disp_r)
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
    left_frame = cv2.imread('src/fyp_wamv_project/fyp_wamv_project/example_images/left_camera_feed.png')
    right_frame = cv2.imread('src/fyp_wamv_project/fyp_wamv_project/example_images/right_camera_feed.png')
    left_frame = cv2.cvtColor(left_frame, cv2.COLOR_BGR2GRAY)
    right_frame = cv2.cvtColor(right_frame, cv2.COLOR_BGR2GRAY)

    # # Without filter
    # stereo = cv2.StereoBM_create(numDisparities=16 * 8, blockSize=7)
    # disparity = stereo.compute(left_frame, right_frame).astype(np.float32) / 16.0
    # plt.imshow(disparity, 'gray')
    # plt.show()
    # depth_map = DepthMap(left_frame,right_frame)
    
    # depth_map.compute_depth_mapBM()
    # # depth_map.compute_depth_mapSGBM()
    # depth_map.plot_images()

    # Test WLS filter
    left_matcher = cv2.StereoBM_create(numDisparities=16 * 8, blockSize=7)
    # left_matcher = cv2.StereoSGBM_create(
    #         minDisparity=16, # Minimum disparity value
    #         numDisparities=16 * 10, # Max disparity - min disparity, must be multiple of 16
    #         blockSize=5, # Must be an odd number >=1, Usually between 3-11
    #         P1= 8 * 3 * 5**2, # Controls disparity smoothness, penalty when disparity difference is 1
    #         P2= 32 * 3 * 5**2, # Controls smoothness, penalty when disparity difference > 1. Rule of thumb: P2 > P1
    #         disp12MaxDiff=1,
    #         uniquenessRatio=20,
    #         speckleWindowSize=200,
    #         speckleRange=2,
    #         mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
    #     )
    right_matcher = cv2.ximgproc.createRightMatcher(left_matcher) # Required for WLS filter
    # Create the WLS filter
    wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=left_matcher)
    wls_filter.setLambda(800) # Smoothing strength
    wls_filter.setSigmaColor(1.2) # How much to respect RGB edges
    # Compute disparity maps
    disp_l = left_matcher.compute(left_frame, right_frame).astype(np.float32) / 16.0
    disp_r = right_matcher.compute(right_frame, left_frame).astype(np.float32) / 16.0
    filtered_disp = wls_filter.filter(disp_l, left_frame, disparity_map_right=disp_r)
    
    plt.imshow(filtered_disp, 'gray')
    plt.show()

if __name__ == "__main__":
    main()
