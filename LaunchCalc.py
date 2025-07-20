import projectilepy as Projectile
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math

PWCD = 0.41
NINECD = 0.39
EIGHTCD = 0.375
SEVENCD = 0.355
SIXCD = 0.35
FIVECD = 0.331
FOURCD = 0.33
THREECD = 0.328
HYBRIDCD = 0.315
FIVEWOODCD = 0.324
THREEWOODCD = 0.28
DRIVERCD = 0.265

class club:
    def __init__(self, name, launch_angle, cd, spin, speed):
        self.name = name
        self.launch_angle = launch_angle
        self.cd = cd
        self.spin = spin
        self.speed = speed

pw = club("Pitching Wedge", 24.2, PWCD, 9304, 102)
nine = club("9 Iron", 20.4, NINECD, 8647, 109)
eight = club("8 Iron", 18.1, EIGHTCD, 7998, 115)
seven = club("7 Iron", 16.3, SEVENCD, 7097, 120)
six = club("6 Iron", 14.1, SIXCD, 6231, 127)
five = club("5 Iron", 12.1, FIVECD, 5361, 132)
four = club("4 Iron", 11.0, FOURCD, 4836, 137)
three = club("3 Iron", 10.4, THREECD, 4630, 142)
hybrid = club("Hybrid", 10.2, HYBRIDCD, 4437, 146)
fivewood = club("5 Wood", 9.4, FIVEWOODCD, 4350, 152)
threewood = club("3 Wood", 9.2, THREEWOODCD, 4655, 158)
driver = club("Driver", 10.9, DRIVERCD, 2686, 167)

def force_drag(rho, v_total, C_D, area):
    '''Returns the scalar magnitude of drag force'''
    return 0.5 * rho * C_D * area * v_total**2

def force_lift(rho, v_total, C_L, area):
    '''Returns the scalar magnitude of lift force'''
    return 0.5 * rho * C_L * area * v_total**2

def lift_trajectory(u, theta_deg, C_D, spin_RPM):
    '''Simulates the golf ball trajectory with drag and lift (Magnus effect)'''
    
    # Constants
    dt = 0.01                # time step (s)
    g = -9.81                # gravity (m/s^2)
    rho = 1.225              # air density (kg/m^3)
    area = 0.00138           # cross-sectional area of golf ball (m^2)
    m = 0.045                # mass of golf ball (kg)

    # Launch angle and initial velocity components
    theta_rad = np.radians(theta_deg)
    v_x = u * np.cos(theta_rad)
    v_y = u * np.sin(theta_rad)
    s_x = 0.0
    s_y = 0.0

    # Estimate lift coefficient based on spin
    # Empirical cap around 0.25 for 2500 RPM
    C_L = min(0.0001 * spin_RPM, 0.25)

    # Arrays to store trajectory
    x_array = [s_x]
    y_array = [s_y]
    t_array = [0]

    t = 0.0

    # Simulation loop
    while s_y >= 0:
        v_total = np.sqrt(v_x**2 + v_y**2)

        # Drag force components (opposes velocity)
        F_D = force_drag(rho, v_total, C_D, area)
        a_Dx = -F_D * (v_x / v_total) / m
        a_Dy = -F_D * (v_y / v_total) / m

        # Lift force components (perpendicular to velocity)
        F_L = force_lift(rho, v_total, C_L, area)
        a_Lx = -F_L * (v_y / v_total) / m   # -vy gives perpendicular
        a_Ly = F_L * (v_x / v_total) / m

        # Total acceleration
        a_x = a_Dx + a_Lx
        a_y = a_Dy + a_Ly + g

        # Update position
        s_x += v_x * dt + 0.5 * a_x * dt**2
        s_y += v_y * dt + 0.5 * a_y * dt**2

        # Update velocity
        v_x += a_x * dt
        v_y += a_y * dt

        # Update time and save results
        t += dt
        x_array.append(s_x)
        y_array.append(s_y)
        t_array.append(t)

    return np.array(x_array), np.array(y_array), np.array(t_array), x_array[-1]# carry distance

"""
def launch_calc(velocity, angle, height=0):
    mySimulator = Projectile.model(initial_velocity=velocity, initial_angle=angle)
    
    mySimulator.drag = "Newtonian"
    mySimulator.mass = 0.046
    mySimulator.drag_coefficient = 0.235
    mySimulator.cross_sectional_area = 0.00145

    mySimulator.run()

    final_position = mySimulator.final_position()
    distance = final_position[0] * 1.09361
    print("Total distance:", distance)

    x, y = zip(*mySimulator.positionValues)
    fig, ax = plt.subplots()
    ax.plot(x, y, label='Projectile Path')
    plt.show()
"""

#launch_calc(velocity=50, angle=45, height=0)

def calculate_spin_loft(dynamic_loft, angle_attack):
    """
    Calculate the spin loft based on dynamic loft and angle of attack.

    Parameters:
    - dynamic_loft (float): Dynamic loft in degrees
    - angle_attack (float): Angle of attack in degrees

    Returns:
    - spin_loft (float): Spin loft in degrees
    """
    if dynamic_loft == 0:
        raise ValueError("Dynamic loft must not be zero.")
    
    spin_loft = dynamic_loft - angle_attack

    return spin_loft

def calculate_spin_axis(face_angle, path_angle, spin_loft):
    if spin_loft == 0:
        raise ValueError("Spin loft must not be zero.")
    axis_rad = math.atan((face_angle - path_angle) / spin_loft)

    return math.degrees(axis_rad)


def calculate_side_spin(total_spin_rpm, spin_axis_deg):
    axis_rad = math.radians(spin_axis_deg)

    return total_spin_rpm * math.sin(axis_rad)

def estimate_lateral_distance(time_of_flight_sec, side_spin_rpm, spin_axis_deg, ball_speed_mph,
                              magnus_coefficient=0.05, max_ball_speed=250, speed_exponent=0.1):
    """
    Estimate lateral distance in golf with nonlinear ball speed scaling.

    Parameters:
    - time_of_flight_sec (float): Time the ball is in the air (seconds)
    - side_spin_rpm (float): Side spin in revolutions per minute (RPM)
    - spin_axis_deg (float): Spin axis in degrees (positive = fade, negative = draw)
    - ball_speed_mph (float): Ball speed in miles per hour
    - magnus_coefficient (float): Base lateral acceleration coefficient (ft/s² per rad/s)
    - max_ball_speed (float): Reference max ball speed for scaling (default 130 mph)
    - speed_exponent (float): Exponent for nonlinear scaling (0 < speed_exponent <= 1), default 0.5 = sqrt

    Returns:
    - lateral_distance_ft (float): Estimated lateral distance in feet
    """

    # Convert side spin RPM to radians/sec
    omega_rad_per_sec = (2 * math.pi * side_spin_rpm) / 60.0

    # Clamp spin axis angle to avoid over-amplification
    max_spin_axis_deg = 45
    spin_axis_deg_clamped = max(min(spin_axis_deg, max_spin_axis_deg), -max_spin_axis_deg)
    spin_axis_rad = math.radians(spin_axis_deg_clamped)

    # Normalize ball speed ratio (0 to 1)
    speed_ratio = ball_speed_mph / max_ball_speed

    # Nonlinear scaling of ball speed
    speed_scale = speed_ratio ** speed_exponent

    # Lateral acceleration scaled by ball speed and spin axis
    lateral_acceleration = magnus_coefficient * omega_rad_per_sec * math.sin(spin_axis_rad) * speed_scale

    # Kinematic estimate for lateral displacement
    lateral_distance_ft = 0.5 * lateral_acceleration * (time_of_flight_sec ** 2)

    return lateral_distance_ft

def graphLiftTrajectory(club_instance, face_angle, path_angle, dynamic_loft, angle_attack):
    '''define a function to graph the trajectory of a ball with lift and drag'''
    
    u = club_instance.speed * 0.44704  # Convert speed from mph to m/s
    angle = club_instance.launch_angle
    CD_dimpled = club_instance.cd
    spin = club_instance.spin
    x, y, t, carry = lift_trajectory(u, angle, CD_dimpled, spin)
    spin_axis = calculate_spin_axis(face_angle, path_angle, calculate_spin_loft(dynamic_loft, angle_attack)) # Assuming face and path angles are zero for simplicity
    side_spin = calculate_side_spin(spin, spin_axis)  # Assuming face and path angles are zero for simplicity
    lateral_distance = estimate_lateral_distance(t[-1], side_spin, spin_axis, u * 2.23694)  # Convert speed to mph for the function

    print(f"Carry distance: {carry:.2f} meters ({carry * 1.094:.1f} yards)")
    print("Lateral distance (yd):", lateral_distance / 3)
    print("Spin axis (degrees):", spin_axis)
    print("Side spin (RPM):", side_spin)

    a_z = 2 * lateral_distance / t[-1]**2
    z_array = 0.5 * a_z * np.array(t)**2

    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(x * 1.094, z_array / 3, y * 1.094, label='3D Golf Ball Trajectory')
    ax.set_xlabel('Forward Distance (x) [yd]')
    ax.set_ylabel('Lateral Distance (z) [yd]')
    ax.set_zlabel('Height (y) [yd]')
    ax.set_title('3D Trajectory of a Golf Ball')
    ax.legend()
    ax.set_xlim(0, 300)
    ax.set_ylim(20, -20)
    ax.set_zlim(0, 60)
    plt.tight_layout()
    plt.show()

graphLiftTrajectory(eight, -5, 0, 10.9, -1.3)
