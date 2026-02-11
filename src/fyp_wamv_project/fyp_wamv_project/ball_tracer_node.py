import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class BallTracer(node):
    def __init__(self):
        super().__init__('balltracer') # Important
        # Subscribe to bridged Pose
        self.subscription = self.create_subscription(
            TFMessage,
            'topic',
            self.callback,
            10
        )
        self.subscription
    
    