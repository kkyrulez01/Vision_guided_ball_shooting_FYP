import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters

from fyp_wamv_project.depth_map import DepthMap

class Dual_ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.bridge = CvBridge()

        # Creat subscribers for left and right camera
        self.left_image_sub = message_filters.Subscriber(self, Image, '/wamv/sensors/cameras/front_left_camera_sensor/image_raw',)
        self.right_image_sub = message_filters.Subscriber(self, Image, '/wamv/sensors/cameras/front_right_camera_sensor/image_raw',)

        # Synchronize the two image topics
        self.ts = message_filters.ApproximateTimeSynchronizer([self.left_image_sub, self.right_image_sub],
                                                            queue_size=10,
                                                            slop=0.1)
        self.ts.registerCallback(self.image_callback)

    def image_callback(self, left_image_msg, right_image_msg): 
        # Store the frames
        self.left_camera_image = self.bridge.imgmsg_to_cv2(left_image_msg, desired_encoding='bgr8')
        self.right_camera_image = self.bridge.imgmsg_to_cv2(right_image_msg, desired_encoding='bgr8')
        self.get_logger().info('Received images from both cameras')

        # Apply depth map computation
        depth_map = DepthMap(self.left_camera_image, self.right_camera_image)
        disparity = depth_map.compute_depth_mapBM()
        # disparity = depth_map.compute_depth_mapSGBM() # Uncomment to use SGBM instead of BM
        disparity_vis = disparity.astype(np.float32) / 16.0
        norm_disparity = cv2.normalize(disparity_vis, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX)

        cv2.imshow("Left_camera_feed", self.left_camera_image)
        cv2.imshow("Right_camera_feed", self.right_camera_image)
        cv2.imshow("Depth_map", norm_disparity)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    dual_cameras_subscriber = Dual_ImageSubscriber()
    rclpy.spin(dual_cameras_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    dual_cameras_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
