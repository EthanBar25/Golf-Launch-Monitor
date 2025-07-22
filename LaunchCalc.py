import projectilepy as Projectile
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import math

PWCD = 0.317
NINECD = 0.319
EIGHTCD = 0.309
SEVENCD = 0.295
SIXCD = 0.296
FIVECD = 0.2815
FOURCD = 0.2776
THREECD = 0.277
HYBRIDCD = 0.267
FIVEWOODCD = 0.2753
THREEWOODCD = 0.271
DRIVERCD = 0.246

class club_data:
    def __init__(self, name, launch_angle, cd, spin, speed, angle_attack):
        self.name = name
        self.launch_angle = launch_angle
        self.cd = cd
        self.spin = spin
        self.speed = speed
        self.angle_attack = angle_attack

pw = club_data("Pitching Wedge", 24.2, PWCD, 9304, 102, -4.7)
nine = club_data("9 Iron", 20.4, NINECD, 8647, 109, -4.3)
eight = club_data("8 Iron", 18.1, EIGHTCD, 7998, 115, -4.2)
seven = club_data("7 Iron", 16.3, SEVENCD, 7097, 120, -3.9)
six = club_data("6 Iron", 14.1, SIXCD, 6231, 127, -3.7)
five = club_data("5 Iron", 12.1, FIVECD, 5361, 132, -3.4)
four = club_data("4 Iron", 11.0, FOURCD, 4836, 137, -2.9)
three = club_data("3 Iron", 10.4, THREECD, 4630, 142, -2.5)
hybrid = club_data("Hybrid", 10.2, HYBRIDCD, 4437, 146, -2.4)
fivewood = club_data("5 Wood", 9.4, FIVEWOODCD, 4350, 152, -2.5)
threewood = club_data("3 Wood", 9.2, THREEWOODCD, 4655, 158, -2.3)
driver = club_data("Driver", 10.9, DRIVERCD, 2686, 167, -0.9)

def force_drag(rho, v_total, C_D, area):
    '''Returns the scalar magnitude of drag force'''
    return 0.5 * rho * C_D * area * v_total**2

def force_lift(rho, v_total, C_L, area):
    '''Returns the scalar magnitude of lift force'''
    return 0.5 * rho * C_L * area * v_total**2

def estimate_rollout_vector(x_landing, z_landing, vx_landing, vy_landing, vz_landing, spin, surface='fairway'):
    """
    Estimate rollout vector after landing, accounting for lateral speed and descent angle.
    
    Parameters:
    - vx_landing (float): Forward velocity (m/s)
    - vy_landing (float): Vertical velocity (m/s)
    - vz_landing (float): Lateral velocity (m/s)
    - surface (str): Type of ground ('fairway', 'rough', 'hard', 'wet')
    
    Returns:
    - rollout_vector_m (np.array): [x_roll, z_roll] in meters
    - rollout_distance_m (float): total rollout distance (2D magnitude)
    """

    g = 9.81

    # Horizontal landing velocity vector
    v_horizontal = np.array([vx_landing, vz_landing])
    v_horiz_mag = np.linalg.norm(v_horizontal)

    # Descent angle (used to scale roll amount)
    descent_angle_rad = math.atan2(abs(vy_landing), v_horiz_mag)

    # Surface friction coefficients (tunable)
    surface_factors = {
        'fairway': 2.5,
        'rough': 4,
        'hard': 0.6,
        'wet': 1.1
    }
    k = surface_factors.get(surface.lower())  # default fallback

    a_friction = k * g

    time = v_horiz_mag / a_friction if a_friction > 0 else 0

    # Base rollout scalar
    rollout_mag = (v_horiz_mag**2) / (2 * a_friction) * math.cos(descent_angle_rad)
    rollout_mag *= 1 - spin / 10000

    # Apply in direction of horizontal velocity
    if v_horiz_mag != 0:
        direction = v_horizontal / v_horiz_mag
    else:
        direction = np.array([0.0, 0.0])

    if time > 0:
        t_vector = np.linspace(0, time, 50)
    else:
        t_vector = np.array([0.0])

    s_t = v_horiz_mag * t_vector - 0.5 * a_friction * t_vector**2
    s_t = np.clip(s_t, 0, None)  # Ensure no negative distances

    x_roll = x_landing + s_t * direction[0]
    z_roll = z_landing + s_t * direction[1]

    rollout_vector_m = np.array([x_roll[-1], z_roll[-1]])
    rollout_distance = np.linalg.norm(rollout_vector_m - np.array([x_landing, z_landing]))

    return x_roll, z_roll, rollout_vector_m, rollout_distance, time

def lift_trajectory(u, theta_deg, C_D, spin_RPM, spin_axis_deg, surface):
    """
    Simulates the 3D golf ball trajectory with drag and lift using spin axis orientation.
    """
    # Constants
    dt = 0.01                # time step (s)
    g = -9.81                # gravity (m/s^2)
    rho = 1.225              # air density (kg/m^3)
    area = 0.00138           # cross-sectional area of golf ball (m^2)
    m = 0.045                # mass of golf ball (kg)

    # Convert angles and velocity
    theta_rad = np.radians(theta_deg)
    spin_axis_rad = np.radians(spin_axis_deg)

    # Initial velocity vector in x (horizontal) and y (vertical), z starts at 0
    v = np.array([
        u * np.cos(theta_rad),  # x
        u * np.sin(theta_rad),  # y
        0.0                     # z
    ])
    
    # Initial position
    s = np.array([0.0, 0.0, 0.0])  # x, y, z

    # Estimate lift coefficient based on spin
    C_L = min(0.0001 * spin_RPM, 0.25)

    # Define spin axis vector (tilt around y-axis)
    spin_axis_vector = np.array([
        np.sin(spin_axis_rad),  # lateral spin component (z-axis rotation)
        0.0,                    # no vertical rotation
        np.cos(spin_axis_rad)   # mostly vertical spin
    ])

    # Lists to store trajectory
    x_array = [s[0]]
    y_array = [s[1]]
    z_array = [s[2]]
    t_array = [0.0]

    t = 0.0

    while s[1] >= 0:  # While above ground
        v_total = np.linalg.norm(v)
        if v_total == 0:
            break

        v_hat = v / v_total

        # Drag force
        F_D = force_drag(rho, v_total, C_D, area)
        a_D = -F_D / m * v_hat

        # Lift force (perpendicular to velocity and aligned with spin axis)
        lift_dir = np.cross(spin_axis_vector, v_hat)
        lift_dir_mag = np.linalg.norm(lift_dir)
        if lift_dir_mag != 0:
            lift_dir /= lift_dir_mag
        else:
            lift_dir = np.zeros(3)

        F_L = force_lift(rho, v_total, C_L, area)
        a_L = F_L / m * lift_dir

        # Total acceleration (add gravity in y direction)
        a = a_D + a_L + np.array([0.0, g, 0.0])

        # Update position and velocity
        s += v * dt + 0.5 * a * dt**2
        v += a * dt
        t += dt

        # Store data
        x_array.append(s[0])
        y_array.append(s[1])
        z_array.append(s[2])
        t_array.append(t)

    vx_landing = v[0]
    vy_landing = v[1]
    vz_landing = v[2]

    x_roll, z_roll, roll_vec, roll_dist, roll_time = estimate_rollout_vector(x_array[-1], z_array[-1], vx_landing, vy_landing, vz_landing, spin_RPM, surface)

    return np.array(x_array), np.array(y_array), np.array(z_array), np.array(t_array), x_array[-1], x_roll, z_roll, roll_vec, roll_dist, roll_time


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
                              magnus_coefficient=0.05):
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

    # Lateral acceleration scaled by ball speed and spin axis
    lateral_acceleration = magnus_coefficient * omega_rad_per_sec * math.sin(spin_axis_rad)

    # Kinematic estimate for lateral displacement
    lateral_distance_ft = 0.5 * lateral_acceleration * (time_of_flight_sec ** 2)

    return lateral_distance_ft * abs(spin_axis_deg) / spin_axis_deg

def calculate_dynamic_loft(launch_angle, angle_attack):
    """
    Calculate the dynamic loft based on launch angle and angle of attack.

    Parameters:
    - launch_angle (float): Launch angle in degrees
    - angle_attack (float): Angle of attack in degrees

    Returns:
    - dynamic_loft (float): Dynamic loft in degrees
    """
    dynamic_loft = (launch_angle + 0.15 * angle_attack) / 0.85

    return dynamic_loft

def adjust_carry_for_lateral(carry_meters, lateral_distance_meters, spin_axis):
    """
    Reduces carry distance due to lateral curvature (slice/hook).
    Slices penalize carry more than hooks due to higher drag.

    Parameters:
    - carry_meters (float): original carry distance
    - lateral_distance_meters (float): lateral deviation
    - spin_axis_deg (float): spin axis (positive = slice, negative = hook)

    Returns:
    - adjusted_carry (float): modified carry distance
    """
    lateral_ratio = abs(lateral_distance_meters) / (carry_meters + 1e-6)

    # More penalty for slice (positive axis), less for draw (negative axis)
    if spin_axis >= 0:
        penalty = 0.15 * lateral_ratio ** 1.2
        adjusted_carry = carry_meters * (1 - penalty)
    elif spin_axis < 0 and spin_axis > -15:
        # Hook: give a small bonus for mild hooks, taper for extremes
        reward = 0.08 * lateral_ratio ** 1.2

        # Cap reward based on spin axis
        axis_abs = abs(spin_axis)
        if axis_abs < 10:
            boost_factor = 1 + reward
        elif axis_abs < 20:
            boost_factor = 1 + 0.5 * reward
        else:
            boost_factor = 1 + 0.2 * reward  # diminishing returns

        adjusted_carry = carry_meters * boost_factor
    elif spin_axis <= -15:
        # Extreme hook: reduce carry slightly
        penalty = 0.15 * lateral_ratio ** 1.2
        adjusted_carry = carry_meters * (1 - penalty)
    else:
        adjusted_carry = carry_meters  # Straight shot

    return adjusted_carry

def graphLiftTrajectory(club_instance, face_angle, path_angle, surface):
    '''define a function to graph the trajectory of a ball with lift and drag'''
    
    u = club_instance.speed * 0.44704  # Convert speed from mph to m/s
    launch_angle = club_instance.launch_angle
    CD_dimpled = club_instance.cd
    spin = club_instance.spin
    angle_attack = club_instance.angle_attack
    dynamic_loft = calculate_dynamic_loft(launch_angle, angle_attack)
    spin_axis = calculate_spin_axis(face_angle, path_angle, calculate_spin_loft(dynamic_loft, angle_attack)) # Assuming face and path angles are zero for simplicity
    side_spin = calculate_side_spin(spin, spin_axis)  # Assuming face and path angles are zero for simplicity
    x, y, z, t, carry_dist, x_roll, z_roll, roll_vec, roll_dist, t_roll = lift_trajectory(u, launch_angle, CD_dimpled, spin, spin_axis, surface)
    lateral_distance = z[-1]
    height = max(y)
    adjusted_carry = adjust_carry_for_lateral(carry_dist, lateral_distance, spin_axis)

    x = x * (adjusted_carry / carry_dist)

    # Append rollout starting from adjusted landing point
    x_tot = 1.094 * np.concatenate([x, x_roll])
    z_tot = 1.094 * np.concatenate([z, z_roll])
    y_tot = 1.094 * np.concatenate([y, np.zeros_like(x_roll)])

    total_dist = np.sqrt(x_tot[-1]**2 + z_tot[-1]**2)

    print(f"Optimal carry distance: {carry_dist:.2f} m ({carry_dist * 1.094:.1f} yd)")
    print(f"Actual carry distance: {adjusted_carry:.2f} m ({adjusted_carry * 1.094:.1f} yd)")
    print(f"Total distance: {total_dist:.2f} m ({total_dist * 1.094:.1f} yd)")
    print(f"Height: {height * 1.094:.2f} yd ({height * 1.094 / 3:.2f} ft)")
    print("Lateral carry distance (yd):", lateral_distance * 1.094)
    print("Spin (RPM):", spin)
    print("Spin axis (degrees):", spin_axis)
    print("Side spin (RPM):", side_spin)

    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(111, projection='3d')

    #ax.plot(x_tot, z_tot, y_tot, label='3D Golf Ball Trajectory')
    ax.set_xlabel('Forward Distance (x) [yd]')
    ax.set_ylabel('Lateral Distance (z) [yd]')
    ax.set_zlabel('Height (y) [yd]')
    ax.set_title('3D Trajectory of a Golf Ball')
    ax.legend()
    ax.set_xlim(0, 350)
    ax.set_ylim(50, -50)
    ax.set_zlim(0, 60)
    plt.tight_layout()

    step = 4  # Reduce number of points for smoother animation
    x_tot = x_tot[::step]
    y_tot = y_tot[::step]
    z_tot = z_tot[::step]

    ball, = ax.plot([], [], [], 'ro', markersize=6)  # Red ball
    trail, = ax.plot([], [], [], 'b-', linewidth=1)  # Blue trail

    def init():
        ball.set_data([], [])
        ball.set_3d_properties([])
        trail.set_data([], [])
        trail.set_3d_properties([])
        return ball, trail

    def update(frame):
        ball.set_data([x_tot[frame]], [z_tot[frame]])  # x and z
        ball.set_3d_properties([y_tot[frame]])       # y (height)
        
        trail.set_data(x_tot[:frame + 1], z_tot[:frame + 1])
        trail.set_3d_properties(y_tot[:frame + 1])

        if frame >= len(x_tot) - 1:
            # Final position
            ball.set_markerfacecolor('g')
            ani.event_source.stop()  # Stop animation at the end

        return ball, trail
    
    ani = FuncAnimation(
    fig,
    update,
    frames=len(x_tot),
    init_func=init,
    blit=False,     # 3D plots don't fully support blitting
    interval=20     # milliseconds between frames
    )

    plt.show()

graphLiftTrajectory(pw, 0, 0, "rough")
