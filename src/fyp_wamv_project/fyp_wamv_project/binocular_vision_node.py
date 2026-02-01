import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters

from fyp_wamv_project.binocular_vision_engine import StereoProcessor

class BinocularVision(Node):
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

        # Template matching
        templates = ['/home/kky/fyp_ws/src/fyp_wamv_project/fyp_wamv_project/templates/filtered_big_target_template.png',
                     '/home/kky/fyp_ws/src/fyp_wamv_project/fyp_wamv_project/templates/filtered_small_target_template.png']

        # Focal length and baseline in meters
        baseline = 0.2 # in meters
        
        stereoProcessor = StereoProcessor(self.left_camera_image, self.right_camera_image, baseline, templates)
        disparity, left_frame_matched = stereoProcessor.process_frames()

        # Display the frames
        cv2.imshow("Left_camera_feed", left_frame_matched)
        # cv2.imshow("Right_camera_feed", right_frame_matched)
        # cv2.imshow("Depth_map", disparity)
        # cv2.imshow("Filtered Left frame", filtered_left_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    binocular_vision = BinocularVision()
    rclpy.spin(binocular_vision)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    binocular_vision.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
