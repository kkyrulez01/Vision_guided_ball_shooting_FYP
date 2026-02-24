import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from collections import deque
import numpy as np

class BallLauncherController(Node):
    def __init__(self):
        super().__init__('ball_launcher_controller')

        # Initialize buffer for X,Y,Z values
        self.window_size = 7
        self.x_buffer = deque(maxlen=self.window_size)
        self.y_buffer = deque(maxlen=self.window_size)
        self.z_buffer = deque(maxlen=self.window_size)

        # Create subscriber to target positions
        self.subscription(
            PointStamped,
            'target_position',
            self.target_position_callback,
            10
        )

    def target_position_callback(self, msg):
        # Extract X,Y,Z values from the message and add to buffer
        self.x_buffer.append(msg.point.x)
        self.y_buffer.append(msg.point.y)
        self.z_buffer.append(msg.point.z)

        # Only calculate mean when buffer is full
        if len(self.z_buffer) == self.window_size:
            mean_x = np.mean(self.x_buffer)
            mean_y = np.mean(self.y_buffer)
            mean_z = np.mean(self.z_buffer)
        
        
