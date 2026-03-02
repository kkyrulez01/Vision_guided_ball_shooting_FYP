import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Float64MultiArray, Float64
from geometry_msgs.msg import PoseArray
from collections import deque
import numpy as np
import time

from fyp_wamv_project.ball_trajectory import *

class BallLauncherController(Node):
    def __init__(self):
        super().__init__('ball_launcher_controller')

        # Initialize buffer for X,Y,Z values
        self.window_size = 10
        self.target_0_history = deque(maxlen=self.window_size)
        self.target_1_history = deque(maxlen=self.window_size)

        # Create subscriber to target positions
        self.subscription = self.create_subscription(
            PoseArray,
            'target_positions', # Subscribe to "target_positions" topic published by binocular vision node
            self.target_position_callback,
            10
        )

        # Create a QoS profile with transient local durability
        latching_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Create publisher to ball shooter base link
        self.ball_shooter_base_publisher = self.create_publisher(Float64MultiArray, '/wamv/base_link_controller/commands', qos_profile=latching_qos)
        # Create publisher to ball shooter link
        self.ball_shooter_link_publisher = self.create_publisher(Float64MultiArray, '/wamv/pitch_controller/commands', qos_profile=latching_qos)

        self.current_state = "IDLE" # To ensure we only fire once per targets detection
        # Callback 2 is triggered 10 times a second
        self.shooter_timer = self.create_timer(0.1, self.trigger_ball_shooter_callback)
        self.has_fired = False

    def target_position_callback(self, msg):
        # If the ball shooter is already aiming or firing , ignore new camera frames until it finishes
        if self.current_state != "IDLE":
            return
        
        # Store yaw and pitch angles in a list
        self.yaw_angles = []
        self.pitch_angles = []
        # Extract X,Y,Z values from the message and add to buffer
        num_targets = len(msg.poses)
        if num_targets == 2: # There should be 2 targets detected
            t0 = msg.poses[0].position
            self.target_0_history.append((t0.x, t0.y, t0.z))

            t1 = msg.poses[1].position
            self.target_1_history.append((t1.x, t1.y, t1.z))

        # Only calculate mean when buffer is full
        if len(self.target_0_history) == self.window_size and len(self.target_1_history) == self.window_size:
            mean_x_0 = np.mean([param[0] for param in self.target_0_history])
            mean_y_0 = np.mean([param[1] for param in self.target_0_history])
            mean_z_0 = np.mean([param[2] for param in self.target_0_history])
            self.get_logger().info(f"Target 0 Mean X: {mean_x_0}, Target 0 Mean Y: {mean_y_0}, Target 0 Mean Z: {mean_z_0}")

            mean_x_1 = np.mean([param[0] for param in self.target_1_history])
            mean_y_1 = np.mean([param[1] for param in self.target_1_history])
            mean_z_1 = np.mean([param[2] for param in self.target_1_history])
            self.get_logger().info(f"Target 1 Mean X: {mean_x_1}, Target 1 Mean Y: {mean_y_1}, Target 1 Mean Z: {mean_z_1}")

            # Get required XYZ for each target relative to ball shooter
            req_x_0, req_y_0, req_z_0 = calculate_required_XYZ(mean_x_0, mean_y_0, mean_z_0)
            req_x_1, req_y_1, req_z_1 = calculate_required_XYZ(mean_x_1, mean_y_1, mean_z_1)

            # Calculate required yaw angle for ball shooter base link to rotate for each target
            req_yaw_angle_0, new_req_z_0 = calculate_required_yaw_angle(req_x_0, req_z_0) # Target 0
            self.get_logger().info(f"Required yaw angle for target 0: {req_yaw_angle_0}")

            req_yaw_angle_1, new_req_z_1 = calculate_required_yaw_angle(req_x_1, req_z_1) # Target 1
            self.get_logger().info(f"Required yaw angle for target 1: {req_yaw_angle_1}")
            
            # Append to yaw_angles list
            self.yaw_angles.append(req_yaw_angle_0)
            self.yaw_angles.append(req_yaw_angle_1)

            # Calculate initial velocity of ball
            initial_velocity = calculate_initial_velocity(shot_force=70, max_step_size=0.004, ball_mass=0.04)
            self.get_logger().info(f"Initial velocity: {initial_velocity}")

            # Calculate required pitch angle for ball shooter to hit each target
            req_pitch_angle_0 = calculate_required_pitch_angle(req_y_0, new_req_z_0, initial_velocity, g=9.81)
            self.get_logger().info(f"Required pitch angle for target 0: {req_pitch_angle_0}")

            req_pitch_angle_1 = calculate_required_pitch_angle(req_y_1, new_req_z_1, initial_velocity, g=9.81)
            self.get_logger().info(f"Required pitch angle for target 1: {req_pitch_angle_1}")

            # Append to pitch_angles list
            self.pitch_angles.append(-req_pitch_angle_0) # Upwards is negative 
            self.pitch_angles.append(-req_pitch_angle_1)

        self.current_state = "AIMING" # Transition to AIMING state after processing target positions

    def trigger_ball_shooter_callback(self):
        if self.current_state == "IDLE": # IDLE state, do nothing
            pass

        elif self.current_state == "AIMING": # AIMING state, publish the required angles to the ball shooter
            if len(self.yaw_angles) > 0 and len(self.pitch_angles) > 0:
                # Publish yaw angles to ball shooter base link
                yaw_msg = Float64MultiArray()
                yaw_msg.data = [self.yaw_angles[0]]
                self.ball_shooter_base_publisher.publish(yaw_msg)
                self.get_logger().info(f"Published yaw angle: {self.yaw_angles[0]}, now rotating...")

                time.sleep(3.0) # Sleep for a while to allow ball shooter base link to finish rotating before publishing pitch angles

                # Publish pitch angles to ball shooter link
                pitch_msg = Float64MultiArray()
                pitch_msg.data = [self.pitch_angles[0]]
                self.ball_shooter_link_publisher.publish(pitch_msg)
                self.get_logger().info(f"Published pitch angle: {self.pitch_angles[0]}, now aiming...")

                time.sleep(3.0)
                self.current_state = "READY_TO_FIRE" # Finish aiming, now transition to READY_TO_FIRE state
            else:
                # List is empty, go back to IDLE
                self.current_state = "IDLE"
        
        elif self.current_state == "READY_TO_FIRE": # READY_TO_FIRE state, trigger the ball shooter to fire
            # Fire ball shooter here
            self.get_logger().info("Triggering ball shooter to fire!")
            self.yaw_angles.pop(0) 
            self.pitch_angles.pop(0)
            self.current_state = "AIMING" # Next target
        
        
def main(args=None):
    rclpy.init(args=args)
    ball_launcher_controller = BallLauncherController()
    rclpy.spin(ball_launcher_controller)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ball_launcher_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        
