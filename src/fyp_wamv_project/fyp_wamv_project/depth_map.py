import cv2 
import numpy as np
import matplotlib.pyplot as plt

class DepthMap:
    def __init__(self, left_frame, right_frame):
        self.left_frame = cv2.cvtColor(left_frame, cv2.COLOR_BGR2GRAY)
        self.right_frame = cv2.cvtColor(right_frame, cv2.COLOR_BGR2GRAY)
    
    def compute_depth_mapBM(self):
        numDisparities_factor = 8 # Adjust this
        blockSize = 5 # Must be odd and more than 3 (Default is 21)
        stereo = cv2.StereoBM_create(numDisparities= 16 * numDisparities_factor, blockSize=blockSize)
        disparity_BM = stereo.compute(self.left_frame, self.right_frame)
        # plt.imshow(disparity, 'gray')
        # plt.show()
        return disparity_BM

    def compute_depth_mapSGBM(self):
        stereo_sgbm = cv2.StereoSGBM_create(
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

        # Compute disparity map
        disparity_sgbm = stereo_sgbm.compute(self.left_frame, self.right_frame).astype(np.float32) / 16.0
        return disparity_sgbm

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
    left_frame = cv2.imread('/home/kky/Pictures/left_camera_feed.png')
    right_frame = cv2.imread('/home/kky/Pictures/right_camera_feed.png')
    depth_map = DepthMap(left_frame,right_frame)
    
    # depth_map.compute_depth_mapBM()
    depth_map.compute_depth_mapSGBM()
    depth_map.plot_images()

if __name__ == "__main__":
    main()
