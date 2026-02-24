import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from collections import deque
import numpy as np

from fyp_wamv_project.ball_trajectory import *

class BallLauncherController(Node):
    def __init__(self):
        super().__init__('ball_launcher_controller')

        # Initialize buffer for X,Y,Z values
        self.window_size = 10
        self.target_0_history = deque(maxlen=self.window_size)
        self.target_1_history = deque(maxlen=self.window_size)

        # Create subscriber to target positions
        self.subscription(
            PoseArray,
            'target_positions', # Subscribe to "target_positions" topic published by binocular vision node
            self.target_position_callback,
            10
        )

        # Create publisher to ball shooter base link
        self.ball_shooter_base_publisher = self.create_publisher(Float64, '/wamv/shooters/ball_shooter/base/pos', 10)
        # Create publisher to ball shooter link
        self.ball_shooter_link_publisher = self.create_publisher(Float64, '/wamv/shooters/ball_shooter/launcher/pos', 10)

        self.has_fired = False # To ensure we only fire once per targets detection

    def target_position_callback(self, msg):
        # Extract X,Y,Z values from the message and add to buffer
        if len(msg.poses) == 2: # There should be 2 targets detected
            t0 = msg.poses[0].position
            self.target_0_history.append((t0.x, t0.y, t0.z))

            t1 = msg.poses[1].position
            self.target_1_history.append((t1.x, t1.y, t1.z))

        # Only calculate mean when buffer is full
        if len(self.target_0_history) == self.window_size and len(self.target_1_history) == self.window_size:
            mean_x = np.mean([param[0] for param in self.target_0_history])
            mean_y = np.mean([param[1] for param in self.target_0_history])
            mean_z = np.mean([param[2] for param in self.target_0_history])
            self.get_logger().info(f"Mean X: {mean_x}, Mean Y: {mean_y}, Mean Z: {mean_z}")
        
        # Calculate initial velocity of ball
        initial_velocity = calculate_initial_velocity(shot_force=70, max_step_size=0.004, ball_mass=0.04)
        self.get_logger().info(f"Initial velocity: {initial_velocity}")

        # Calculate required pitch angle for ball shooter
        req_pitch_angle = calculate_required_pitch_angle(mean_x, mean_y, mean_z, initial_velocity, g=9.81)

        # Publish required x to move
        # Publish the required pitch angle to the ball shooter link
        if not self.has_fired:
            self.ball_shooter_
        
        
