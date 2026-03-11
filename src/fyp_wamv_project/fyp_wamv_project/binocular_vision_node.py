import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, PoseArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters
import argparse
import sys

from fyp_wamv_project.binocular_vision_engine import BaseDetector, TemplateMatcher, ColorEdgeDetector
from fyp_wamv_project.shape_detection import detect_shape
from fyp_wamv_project.HSV_filter import add_HSV_filter

class BinocularVision(Node):
    def __init__(self, mode):
        super().__init__('binocular_vision')
        self.bridge = CvBridge()

        # Store mode argument (Default: 2)
        self.mode = mode

        # Create subscribers to left and right camera
        self.left_image_sub = message_filters.Subscriber(self, Image, '/wamv/sensors/cameras/front_left_camera_sensor/image_raw',)
        self.right_image_sub = message_filters.Subscriber(self, Image, '/wamv/sensors/cameras/front_right_camera_sensor/image_raw',)

        # Synchronize the two image topics
        self.ts = message_filters.ApproximateTimeSynchronizer([self.left_image_sub, self.right_image_sub],
                                                            queue_size=10,
                                                            slop=0.1)
        self.ts.registerCallback(self.image_callback)

        # Create publisher
        self.targets_publisher = self.create_publisher(PoseArray, 'target_positions', 10) # Publishing to "target_positions" topic

    def image_callback(self, left_image_msg, right_image_msg): 
        # Store the frames
        self.left_camera_image = self.bridge.imgmsg_to_cv2(left_image_msg, desired_encoding='bgr8')
        self.right_camera_image = self.bridge.imgmsg_to_cv2(right_image_msg, desired_encoding='bgr8')
        self.get_logger().info('Received images from both cameras')

        # # Uncomment to show left and right camera frames
        # combined_frame = np.hstack((self.left_camera_image, self.right_camera_image))
        # h, w = self.left_camera_image.shape[:2]
        # cv2.line(combined_frame, (w, 0), (w, h), (0, 0, 0), 1) # Black line to seperate the 2 camera frames
        # cv2.imshow('Dual camera feed', combined_frame)
        # cv2.waitKey(1)

        # Template matching
        templates = ['src/fyp_wamv_project/fyp_wamv_project/templates/small_target_template_1.png',
                    'src/fyp_wamv_project/fyp_wamv_project/templates/small_target_template_2.png',
                    'src/fyp_wamv_project/fyp_wamv_project/templates/small_target_template_3.png']
                    
        # Focal length and baseline in meters
        baseline = 0.2 # in meters
        
        processor1 = TemplateMatcher(self.left_camera_image, self.right_camera_image, templates)
        processor2 = ColorEdgeDetector(self.left_camera_image, self.right_camera_image, baseline)
        
        # Choose approach depending on arguments, default mode is 2
        # Template matching
        if self.mode == 1:
            disparity_1, filtered_frame_1 = processor1.process_frames()

            cv2.imshow("Template matching", filtered_frame_1)
            cv2.waitKey(1)

        # Color edge detection
        else:
            disparity_2, filtered_frame_2, targets = processor2.process_frames()

            # Display the processed frame
            cv2.imshow("Color Edge detection", filtered_frame_2)
            cv2.waitKey(1)

        # # Display the processed_frame
        # cv2.imshow("Left_camera_feed", filtered_left_frame)
        # # cv2.imshow("Right_camera_feed", right_frame_matched)
        # cv2.imshow("Depth_map", disparity)
        # cv2.imshow("Filtered Left frame", filtered_left_frame)
        # cv2.imshow("Template matching", frame_1)
        
        msg = PoseArray()
        for target in targets:
            if target[0] is not None and target[1] is not None and target[2] is not None:
                try:
                    p = Pose()
                    p.position.x = float(target[0])
                    p.position.y = float(target[1])
                    p.position.z = float(target[2])
                    msg.poses.append(p)
                except (TypeError, ValueError) as e:
                    self.get_logger().error(f"Conversion failed: {e}")

            else:
                self.get_logger().info("No target detected, skipping publishing")
        
        # Publish the message
        self.targets_publisher.publish(msg)

def main(args=None):
    # Create a parser to allow user to choose which method to use
    # 1: Template matching 2:  Color Edge detection
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', type=int, default = 2)
    # ROS2 passes hidden arguments that argparse doesn't recognize
    # parse_known_args() captures only our arguments and ignores the ROS2 arguments
    parsed_args, unknown = parser.parse_known_args() 

    rclpy.init(args=args)
    binocular_vision = BinocularVision(mode=parsed_args.mode)
    rclpy.spin(binocular_vision)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    binocular_vision.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
