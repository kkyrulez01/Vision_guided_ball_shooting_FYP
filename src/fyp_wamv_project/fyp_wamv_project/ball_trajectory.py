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

def calculate_required_pitch_angle(mean_x, mean_y, mean_z, initial_velocity, g=9.81):
    # Default front left camera position:
        left_cam_x = 0.1
        left_cam_y = 1.5
        left_cam_z = 0.75

        # Default ball shooter position and pitch
        default_ball_shooter_x = 0.3
        default_ball_shooter_y = 1.4
        default_ball_shooter_z = 0.54

        # Req x,y,z 
        req_x = mean_x - (default_ball_shooter_x - left_cam_x)
        req_y = np.abs(mean_y - (default_ball_shooter_y - left_cam_y))
        req_z = np.abs(mean_z - (default_ball_shooter_z - left_cam_z))

        # Use kinematics to calculate the required pitch angle for ball shooter
        theta_1 = np.arcsin(np.sqrt((2*g*req_y) / initial_velocity**2)) # Using max height
        # theta_2 = 0.5 * np.arcsin((g*req_z) / (2 * initial_velocity**2)) # Using range

        theta_1_deg = np.degrees(theta_1)
        # theta_2_deg = np.degrees(theta_2)

        return theta_1
        
def main():
    ball_mass = 0.04
    max_step_size = 0.004
    shot_force = 100
    initial_velocity = calculate_initial_velocity(shot_force, max_step_size, ball_mass)
    print(f"Initial velocity: {initial_velocity}")
    max_range = get_max_range(initial_velocity, 9.81, 45/180 * np.pi)
    max_height = get_max_height(initial_velocity, 9.81, 45/180 * np.pi)
    print(f"Max range: {max_range}, Max height: {max_height}")
    theta_1 = calculate_required_pitch_angle((0.1-0.12), (1.5+0.34), (4.6-0.75), initial_velocity, g=9.81)
    theta_1_deg = np.degrees(theta_1)
    print(f"theta_1: {theta_1} radians, theta_1_deg: {theta_1_deg}degrees")

if __name__ == '__main__':
    main()