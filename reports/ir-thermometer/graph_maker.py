import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit
import os

# Get the directory of this script
script_dir = os.path.dirname(os.path.abspath(__file__))

# Read the CSV data
data_path = os.path.join(script_dir, 'data', 'measurement1.csv')
df = pd.read_csv(data_path)

# Get IR and NTC temperature values
ir_temp = df['IR Temperature'].values
ntc_temp = df['NTC Temperature'].values

# Calculate the difference (NTC - IR)
diff_values = ntc_temp - ir_temp

# Define fitting functions
def linear_func(x, a, b):
    return a * x + b

def polynomial_func(x, a, b, c):
    return a * x**2 + b * x + c

# Fit polynomial to: NTC = f(IR)
# This gives us a function to predict NTC temperature from IR temperature
popt_ntc, _ = curve_fit(polynomial_func, ir_temp, ntc_temp)
fitted_ntc = polynomial_func(ir_temp, *popt_ntc)

# Also fit linear for comparison
popt_lin, _ = curve_fit(linear_func, ir_temp, ntc_temp)
fitted_linear = linear_func(ir_temp, *popt_lin)

# Fit the difference as function of IR temperature
popt_diff, _ = curve_fit(polynomial_func, ir_temp, diff_values)
fitted_diff = polynomial_func(ir_temp, *popt_diff)

# Calculate R-squared for polynomial fit (NTC vs IR)
ss_res = np.sum((ntc_temp - fitted_ntc) ** 2)
ss_tot = np.sum((ntc_temp - np.mean(ntc_temp)) ** 2)
r_squared = 1 - (ss_res / ss_tot)

# Calculate R-squared for difference fit
ss_res_diff = np.sum((diff_values - fitted_diff) ** 2)
ss_tot_diff = np.sum((diff_values - np.mean(diff_values)) ** 2)
r_squared_diff = 1 - (ss_res_diff / ss_tot_diff)

# Create smooth curve for plotting
ir_smooth = np.linspace(ir_temp.min(), ir_temp.max(), 200)
ntc_fitted_smooth = polynomial_func(ir_smooth, *popt_ntc)
diff_fitted_smooth = polynomial_func(ir_smooth, *popt_diff)

# Create the plot with two subplots
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))

# First plot: NTC vs IR Temperature with fitted curve
ax1.scatter(ir_temp, ntc_temp, label='Measured Data', color='blue', s=20, alpha=0.7)
ax1.plot(ir_smooth, ntc_fitted_smooth, 
         label=f'Fit: NTC = {popt_ntc[0]:.6f}·IR² + {popt_ntc[1]:.4f}·IR + {popt_ntc[2]:.2f}\n(R²={r_squared:.4f})', 
         color='red', linewidth=2)
ax1.plot([ir_temp.min(), ir_temp.max()], [ir_temp.min(), ir_temp.max()], 
         'g--', label='y = x (perfect match)', alpha=0.5)
ax1.set_xlabel('IR Temperature (°C)')
ax1.set_ylabel('NTC Temperature (°C)')
ax1.set_title('NTC Temperature as Function of IR Temperature')
ax1.legend()
ax1.grid(True, linestyle='--', alpha=0.7)

# Second plot: Temperature difference vs IR Temperature
ax2.scatter(ir_temp, diff_values, label='Measured Difference (NTC - IR)', color='green', s=20, alpha=0.7)
ax2.plot(ir_smooth, diff_fitted_smooth, 
         label=f'Fit: Diff = {popt_diff[0]:.6f}·IR² + {popt_diff[1]:.4f}·IR + {popt_diff[2]:.2f}\n(R²={r_squared_diff:.4f})', 
         color='red', linewidth=2)
ax2.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
ax2.set_xlabel('IR Temperature (°C)')
ax2.set_ylabel('Temperature Difference (°C)')
ax2.set_title('Temperature Difference (NTC - IR) vs IR Temperature')
ax2.legend()
ax2.grid(True, linestyle='--', alpha=0.7)

plt.tight_layout()

# Save the graph
graphs_dir = os.path.join(script_dir, 'graphs')
os.makedirs(graphs_dir, exist_ok=True)
output_path = os.path.join(graphs_dir, 'temperature_comparison.png')
plt.savefig(output_path, dpi=150, bbox_inches='tight')
plt.close()

print(f"Graph saved to: {output_path}")
print(f"\n=== Calibration Function ===")
print(f"To convert IR temperature to NTC temperature:")
print(f"NTC = {popt_ntc[0]:.6f} × IR² + {popt_ntc[1]:.6f} × IR + {popt_ntc[2]:.6f}")
print(f"R² = {r_squared:.6f}")
print(f"\nValid range: IR Temperature from {ir_temp.min():.1f}°C to {ir_temp.max():.1f}°C")

# Example usage
print(f"\n=== Example Conversions ===")
for test_ir in [30, 50, 70, 90]:
    predicted_ntc = polynomial_func(test_ir, *popt_ntc)
    print(f"IR = {test_ir}°C  →  NTC ≈ {predicted_ntc:.1f}°C")
