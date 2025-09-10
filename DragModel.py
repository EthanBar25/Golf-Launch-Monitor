import pandas as pd
import numpy as np
import math
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
from sklearn.ensemble import RandomForestRegressor
from sklearn.linear_model import LinearRegression
from sklearn.metrics import r2_score, mean_squared_error
from scipy.optimize import minimize
from tqdm import tqdm

from LaunchCalc import lift_trajectory, calculate_dynamic_loft, calculate_spin_axis, calculate_side_spin, calculate_spin_loft

df = pd.read_csv('golfdata.csv')
df.columns = df.columns.str.strip().str.lower()

X = df[['ball_speed', 'spin_rpm', 'launch_angle', 'attack_angle']].values
Y = []
Z = []

cd_range = np.arange(0.15, 0.32, 0.03)
cl_range = np.arange(0.10, 0.30, 0.03)


def loss_function_cd(true_carry, true_apex, sim_carry, sim_apex):
    return (true_carry - sim_carry) ** 2 + 0.002 * (true_apex - sim_apex) ** 2

def loss_function_cl(true_carry, true_apex, sim_carry, sim_apex):
    return (true_carry - sim_carry) ** 2 + 0.3 * (true_apex - sim_apex) ** 2

def loss_cd_cl(params, ball_speed, launch_angle, spin_RPM, spin_axis, true_carry, true_apex):
    Cd, Cl = params
    _, y, _, _, sim_carry, *_ = lift_trajectory(ball_speed, launch_angle, Cd, Cl, spin_RPM, spin_axis)

    sim_apex = max(y) * 1.094  # Convert to yards
    
    return ((true_carry - sim_carry))**2 + ((true_apex - sim_apex))**2

for n in tqdm(range(len(df))):
    ball_speed, spin_RPM, launch_angle, attack_angle = X[n]
    true_carry = df.loc[n, 'carry_distance']
    true_apex = df.loc[n, 'max_height']
    
    best_loss_cd = float('inf')
    best_loss_cl = float('inf')

    dynamic_loft = calculate_dynamic_loft(launch_angle, attack_angle)
    spin_axis = calculate_spin_axis(0, 0, calculate_spin_loft(dynamic_loft, attack_angle)) # Assuming face and path angles are zero for simplicity
    side_spin = calculate_side_spin(spin_RPM, spin_axis)

    """
    res = minimize(
        loss_cd_cl,
        x0=[0.215, 0.16],
        args=(ball_speed, launch_angle, spin_RPM, spin_axis, true_carry, true_apex),
        bounds=[(0.18, 0.30), (0.1, 0.28)],
        method="L-BFGS-B",
        options={'maxiter': 20}
    )

    best_cd, best_cl = res.x

    """
    for Cd in cd_range:  # Try Cd from 0.2 to 0.4
        for Cl in cl_range: 
            x, y, z, t, sim_carry, x_roll, z_roll, roll_vec, roll_dist, t_roll = lift_trajectory(ball_speed, launch_angle, Cd, Cl, spin_RPM, spin_axis)
        
            sim_apex = max(y) * 1.094
            
            loss_cd = loss_function_cd(true_carry, true_apex, sim_carry, sim_apex)
            loss_cl = loss_function_cl(true_carry, true_apex, sim_carry, sim_apex)
            if loss_cd < best_loss_cd:
                best_cd = Cd
                best_loss_cd = loss_cd

            if loss_cl < best_loss_cl:
                best_cl = Cl
                best_loss_cl = loss_cl

    Y.append(best_cd)
    Z.append(best_cl)

model_cd = RandomForestRegressor(n_estimators=200, max_depth=10, random_state=42)
model_cl = RandomForestRegressor(n_estimators=200, max_depth=10, random_state=42)
model_cd.fit(X, Y)
model_cl.fit(X, Z)
