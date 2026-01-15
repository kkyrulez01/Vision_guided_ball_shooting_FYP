import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/wamv/sensors/cameras/front_left_camera_sensor/image_raw',
            self.listener_callback,
            10)

        self.subscription # prevent unused variable warning
        self.bridge = CvBridge()
    
    def listener_callback(self, msg):
        self.get_logger().info('Received image')
        current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') # Convert ROS Image to OpenCV format
        cv2.imshow("Camera feed", current_frame) # Show the image in a window
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()