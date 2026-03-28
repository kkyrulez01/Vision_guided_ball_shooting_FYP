import numpy as np
import scipy as scp
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

def calculate_required_XYZ(x, y, z):
    # Default front left camera position:
    left_cam_x = 0.1
    left_cam_y = 1.5
    left_cam_z = 0.75

    # Default ball shooter position
    default_ball_shooter_x = 0.3
    default_ball_shooter_y = 1.40388
    default_ball_shooter_z = 0.5196

    # Calculate req X,Y,Z
    req_x = x + (default_ball_shooter_x - left_cam_x)
    req_y = np.abs(y) + (left_cam_y - default_ball_shooter_y)
    req_z = z + (left_cam_z - default_ball_shooter_z)

    return req_x, req_y, req_z

def calculate_required_yaw_angle(req_x, req_z):
        # Use trigonometry to calculate required yaw angle for ball shooter base link
        yaw_angle = np.arctan2(req_x , req_z)
        new_req_z = np.sqrt(req_x**2 + req_z**2)

        return yaw_angle, new_req_z
        
def calculate_required_pitch_angle(req_y, req_z, initial_velocity, g=9.81):
        # Use kinematics to calculate the required pitch angle for ball shooter
        d = req_z
        h = req_y
        u = initial_velocity # To factor for sim-to-real gap
        
        coeff = [(g*d**2), (-2*d*u**2), (2*h*u**2 + g*d**2)]
        possible_theta = np.arctan(np.roots(coeff))
        # Account for default 15 deg angle of ball shooter
        for theta in possible_theta:
            theta = theta - np.deg2rad(15)
        # 1st theta is for a high angle trajectory, 2nd is for a flatter trajectory
        theta_1, theta_2 = possible_theta[0], possible_theta[1]
        # Use the 2nd theta
        return theta_2
        
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
    theta_high= calculate_required_pitch_angle((0.9+1.5-1.40388), (4.5+0.75-0.5196), initial_velocity, g=9.81)
    print(theta_high)
    # print(f"theta_1: {theta_1} radians, theta_1_deg: {np.degrees(theta_1)} degrees")
    # print(f"theta_2: {theta_2} radians, theta_2_deg: {np.degrees(theta_2)} degrees")
    # theta_deg = np.degrees(theta)
    # print(f"theta: {theta} radians, theta_deg: {theta_deg} degrees")
    # yaw_angle, new_req_z = calculate_required_yaw_angle(req_x=-0.32-(0.3-0.1),req_z=4.5+0.75-0.5196)
    # print(f"yaw_angle = {yaw_angle}, new_req_z = {new_req_z}")

if __name__ == '__main__':
    main()