import numpy as np

def calculate_initial_velocity(shot_force, max_step_size, ball_mass):
    initial_velocity = (shot_force * max_step_size) / ball_mass
    return initial_velocity

def get_max_range(initial_velocity, acceleration, pitch_angle):
    # Calculate the max range of the ball
    v0 = initial_velocity
    a = acceleration
    theta = pitch_angle

    max_range = (v0**2 * np.sin(2*theta)) / (a)
    return max_range

def get_max_height(initial_velocity, acceleration, pitch_angle):
    # Calculate the max height of the ball
    v0 = initial_velocity
    a = acceleration
    theta = pitch_angle

    max_height = (v0**2 * np.sin(theta)**2) / (2 * a)
    return max_height

def calculate_required_XYZ(mean_x, mean_y, mean_z):
    # Default front left camera position:
        left_cam_x = 0.1
        left_cam_y = 1.5
        left_cam_z = 0.75

        # Default ball shooter position
        default_ball_shooter_x = 0.3
        default_ball_shooter_y = 1.4
        default_ball_shooter_z = 0.54

        # Req x,y,z 
        req_x = mean_x - (default_ball_shooter_x - left_cam_x)
        req_y = np.abs(mean_y - (default_ball_shooter_y - left_cam_y))
        req_z = np.abs(mean_z - (default_ball_shooter_z - left_cam_z))

        return req_x, req_y, req_z

def calculate_required_yaw_angle(req_x, req_z):
        # Use trigonometry to calculate required yaw angle for ball shooter base link
        yaw_angle = np.arctan(req_x / req_z)
        new_req_z = np.sqrt(req_x**2 + req_z**2)

        return yaw_angle, new_req_z
        
def calculate_required_pitch_angle(req_y, req_z, initial_velocity, g=9.81):
        # Use kinematics to calculate the required pitch angle for ball shooter
        # Condition 1: The ball must reach the required height (req_y)
        # Condition 2: The ball must reach the required range (req_z)
        theta_1 = np.arcsin(np.sqrt((2*g*req_y) / initial_velocity**2)) # Using max height
        theta_2 = 0.5 * np.arcsin((g*req_z) / (2 * initial_velocity**2)) # Using range

        theta = max(theta_1, theta_2) # Return the larger angle to ensure both conditions met
        return theta 
        
def main():
    # Test example
    ball_mass = 0.04
    max_step_size = 0.004
    shot_force = 100
    initial_velocity = calculate_initial_velocity(shot_force, max_step_size, ball_mass)
    print(f"Initial velocity: {initial_velocity}")
    max_range = get_max_range(initial_velocity, 9.81, 45/180 * np.pi)
    max_height = get_max_height(initial_velocity, 9.81, 45/180 * np.pi)
    print(f"Max range: {max_range}, Max height: {max_height}")
    theta= calculate_required_pitch_angle((1.5+0.34), (4.6-0.75), initial_velocity, g=9.81)
    theta_deg = np.degrees(theta)
    print(f"theta: {theta} radians, theta_deg: {theta_deg} degrees")

if __name__ == '__main__':
    main()