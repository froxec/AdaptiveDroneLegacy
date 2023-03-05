import pandas as pd
from IPython.display import display
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
PATH = "./T-motor-2216-900kv.csv"

def func(x, Kt):
    return Kt*x**2
def fit_data_for_rpm(rpm, data, data_name, test_data_range=[0, 900]):
    rpm_to_rad_per_s_coef = 2 * np.pi / 60
    rot_speed_scaled = rpm*rpm_to_rad_per_s_coef
    test_points = np.arange(test_data_range[0], test_data_range[1], 1)
    popt, _ = curve_fit(func, rot_speed_scaled, data)
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.plot(rot_speed_scaled, data, 'o', label='Dane')
    ax.plot(test_points, popt[0] * test_points ** 2, label="interpolation")
    ax.set_xlabel("Prędkość obrotowa [rad/s]")
    ax.set_ylabel(data_name)
    plt.show()
    return popt[0]

if __name__ == "__main__":
    df = pd.read_csv(PATH, sep='\t')
    Kt = fit_data_for_rpm(df['Rotation speed (rpm)'], df['Thrust (kgf)'], 'Thrust (kgf)')
    print("Kt", Kt)
    Kd = fit_data_for_rpm(df['Rotation speed (rpm)'], df['Torque (N⋅m)'], 'Torque (N⋅m)')
    print("Kt", Kd)