import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters

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
        self.ts.registerCallback(self.callback)

    def callback(self, left_image_msg, right_image_msg):
        # Convert ROS Image messages to OpenCV format
        left_camera_image = self.bridge.imgmsg_to_cv2(left_image_msg, desired_encoding='bgr8')
        right_camera_image = self.bridge.imgmsg_to_cv2(right_image_msg, desired_encoding='bgr8')

        # Resize the images
        resized_left_camera_image = cv2.resize(left_camera_image, (640,480))
        resized_right_camera_image = cv2.resize(right_camera_image, (640,480))

        # Stack images horizontally
        dual_images = cv2.hconcat([resized_left_camera_image, resized_right_camera_image])
        cv2.imshow("Dual Camera feed", dual_images)
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
