import os
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit

# Path setup
base_dir = os.path.dirname(__file__)
data_path = os.path.join(base_dir, 'data', 'AutomatedTeaBrewerHeatDissipationTest1.xls')

graphs_dir = os.path.join(base_dir, 'graphs')
os.makedirs(graphs_dir, exist_ok=True)

# Parse the custom .xls (actually tab-separated text)
def parse_data_file(filepath):
    with open(filepath, encoding='utf-16') as f:
        lines = f.readlines()

    # Find where the data starts
    data_start = None
    for i, line in enumerate(lines):
        if line.strip().lower().startswith('no.'):
            data_start = i + 1
            break

    data = []
    if data_start is not None:
        for line in lines[data_start:]:
            parts = line.strip().split('\t')
            if len(parts) >= 3:
                try:
                    idx, t, v = parts[:3]
                    v = float(v.replace(',', '.'))
                    data.append((t, v))
                except Exception:
                    continue
    return list(zip(*data)) if data else ([], [])

# Read and plot

x, y = parse_data_file(data_path)
import datetime

# Convert time strings to hours from the first data point
def parse_time_to_hours(time_strings):
    if not time_strings:
        return []
    fmt = '%Y-%m-%d %H:%M:%S'
    t0 = datetime.datetime.strptime(time_strings[0], fmt)
    hours = []
    for tstr in time_strings:
        t = datetime.datetime.strptime(tstr, fmt)
        delta = t - t0
        hours.append(delta.total_seconds() / 3600)
    return hours

x_hours = np.array(parse_time_to_hours(x))
y = np.array(y)

# Exponential decay function: y = a * exp(-b * x) + c
def exp_decay(x, a, b, c):
    return a * np.exp(-b * x) + c

# Inverse function: given temperature, return time
def exp_decay_inverse(temp, a, b, c):
    """
    Solve for x: temp = a * exp(-b * x) + c
    => exp(-b * x) = (temp - c) / a
    => -b * x = ln((temp - c) / a)
    => x = -ln((temp - c) / a) / b
    """
    if a == 0 or b == 0:
        return None
    arg = (temp - c) / a
    if arg <= 0:
        return None  # Invalid for logarithm
    return -np.log(arg) / b

# Initial guess for parameters: a, b, c
guess = [y[0] - y[-1], 1.0, y[-1]]
try:
    popt, pcov = curve_fit(exp_decay, x_hours, y, p0=guess, maxfev=10000)
    y_fit = exp_decay(x_hours, *popt)
    fit_label = f"Fit: y = {popt[0]:.2f} * exp(-{popt[1]:.2f} * x) + {popt[2]:.2f}"
    fit_success = True
except Exception as e:
    y_fit = None
    fit_label = "Fit failed"
    fit_success = False
    popt = None

# Create two subplots: Temperature vs Time and Time vs Temperature
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6), dpi=800)

# LEFT PLOT: Temperature vs Time (original)
ax1.plot(x_hours, y, marker='o', markersize=3, color='royalblue', label='Measured')
if y_fit is not None:
    ax1.plot(x_hours, y_fit, '--', linewidth=2, color='red', label=fit_label)
ax1.set_xlabel('Time [hours]')
ax1.set_ylabel('Temperature [°C]')
ax1.set_title('Temperature vs Time')
ax1.grid(True)
ax1.legend()
# Annotate the first timestamp on the plot
if len(x) > 0:
    first_time = x[0]
    ax1.annotate(f"Start: {first_time}", xy=(0.01, 0.01), xycoords='axes fraction', 
                fontsize=9, color='dimgray', ha='left', va='bottom', 
                bbox=dict(boxstyle='round,pad=0.2', fc='white', ec='gray', alpha=0.7))

# RIGHT PLOT: Time vs Temperature (inverse function)
if fit_success and popt is not None:
    # Create temperature range for inverse plot
    temp_min = max(popt[2] + 1, np.min(y))  # Slightly above asymptote
    temp_max = np.max(y)
    temp_range = np.linspace(temp_min, temp_max, 100)
    
    # Calculate time for each temperature
    time_for_temp = [exp_decay_inverse(t, *popt) for t in temp_range]
    time_for_temp = [t if t is not None and t >= 0 else np.nan for t in time_for_temp]
    
    # Create inverse function label with equation
    inverse_label = f"Inverse: t = -ln((T - {popt[2]:.2f}) / {popt[0]:.2f}) / {popt[1]:.2f}"
    
    # Plot inverse function
    ax2.plot(temp_range, time_for_temp, '-', linewidth=2, color='green', label=inverse_label)
    
    # Plot measured data points (inverted axes)
    ax2.plot(y, x_hours, 'o', markersize=3, color='royalblue', alpha=0.3, label='Measured data')
    
    ax2.set_xlabel('Target Temperature [°C]')
    ax2.set_ylabel('Time Required [hours]')
    ax2.set_title('Time Required to Reach Target Temperature')
    ax2.grid(True)
    ax2.legend()
    
    # Add example annotations for common temperatures
    example_temps = [30, 40, 50, 60, 70, 80]
    for temp in example_temps:
        if temp_min <= temp <= temp_max:
            time_needed = exp_decay_inverse(temp, *popt)
            if time_needed is not None and time_needed >= 0 and time_needed <= np.max(x_hours):
                ax2.plot(temp, time_needed, 'ro', markersize=6)
                ax2.annotate(f'{temp}°C: {time_needed:.2f}h', 
                           xy=(temp, time_needed), 
                           xytext=(5, 5), textcoords='offset points',
                           fontsize=8, color='darkred')
else:
    ax2.text(0.5, 0.5, 'Inverse function unavailable\n(fit failed)', 
            ha='center', va='center', transform=ax2.transAxes, fontsize=14)
    ax2.set_xlabel('Target Temperature [°C]')
    ax2.set_ylabel('Time Required [hours]')
    ax2.set_title('Time Required to Reach Target Temperature')

plt.tight_layout()
output_path = os.path.join(graphs_dir, 'heat_dissipation_plot.png')
plt.savefig(output_path)
#plt.show()
plt.close()
print(f'Graph saved to {output_path}')

# Print fit equations
if fit_success and popt is not None:
    print("\n=== Fitted Equations ===")
    print(f"Temperature decay: T(t) = {popt[0]:.4f} * exp(-{popt[1]:.4f} * t) + {popt[2]:.4f}")
    print(f"Time required: t(T) = -ln((T - {popt[2]:.4f}) / {popt[0]:.4f}) / {popt[1]:.4f}")
    print("=" * 60)