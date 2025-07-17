import projectilepy as Projectile
import numpy as np
import matplotlib.pyplot as plt

import numpy as np

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

    return np.array(x_array), np.array(y_array), np.array(t_array), x_array[-1]  # carry distance


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

#launch_calc(velocity=50, angle=45, height=0)

def graphLiftTrajectory(u, angle, spin, CD_dimpled):
    '''define a function to graph the trajectory of a ball with lift and drag'''
    
    x, y, t, carry = lift_trajectory(u, angle, CD_dimpled, spin)
    
    print(f"Carry distance: {carry:.2f} meters ({carry * 1.094:.1f} yards)")

    fig, ax = plt.subplots()
    ax.plot(x, y)
    ax.set_xlabel('Distance (m)')
    ax.set_ylabel('Height (m)')
    ax.set_title('Projectile Motion with Lift and Drag')
    plt.show()

driver_cd = 0.265
pw_cd = 0.41
nine_cd = 0.39
eight_cd = 0.375

graphLiftTrajectory(51.41, 18.1, 7998, 0.375)