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

# Initial guess for parameters: a, b, c
guess = [y[0] - y[-1], 1.0, y[-1]]
try:
    popt, pcov = curve_fit(exp_decay, x_hours, y, p0=guess, maxfev=10000)
    y_fit = exp_decay(x_hours, *popt)
    fit_label = f"Fit: y = {popt[0]:.2f} * exp(-{popt[1]:.2f} * x) + {popt[2]:.2f}"
except Exception as e:
    y_fit = None
    fit_label = "Fit failed"

plt.figure(figsize=(10, 6),  dpi=800)
# Use blue for measured, orange for fit
plt.plot(x_hours, y, marker='o', markersize=3, color='royalblue', label='Measured')
if y_fit is not None:
    plt.plot(x_hours, y_fit, '--', linewidth=2, color='red', label=fit_label)
plt.xlabel('Time [hours]')
plt.ylabel('Temperature [°C]')
plt.title('Automated Tea Brewer Heat Dissipation Test')
plt.grid(True)
plt.legend()
# Annotate the first timestamp on the plot
if len(x) > 0:
    first_time = x[0]
    plt.annotate(f"Start: {first_time}", xy=(0.01, 0.01), xycoords='axes fraction', fontsize=9, color='dimgray', ha='left', va='bottom', bbox=dict(boxstyle='round,pad=0.2', fc='white', ec='gray', alpha=0.7))
output_path = os.path.join(graphs_dir, 'heat_dissipation_plot.png')
plt.savefig(output_path)
#plt.show()
plt.close()
print(f'Graph saved to {output_path}')