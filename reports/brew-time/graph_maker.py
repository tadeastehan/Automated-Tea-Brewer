import os
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit
import datetime

# Path setup
base_dir = os.path.dirname(__file__)
data_path = os.path.join(base_dir, 'data', 'brew1.xls')

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


# Convert time strings to seconds from the first data point
def parse_time_to_seconds(time_strings):
    if not time_strings:
        return []
    fmt = '%Y-%m-%d %H:%M:%S'
    t0 = datetime.datetime.strptime(time_strings[0], fmt)
    seconds = []
    for tstr in time_strings:
        t = datetime.datetime.strptime(tstr, fmt)
        delta = t - t0
        seconds.append(delta.total_seconds())
    return seconds


# Exponential growth function: y = a * (1 - exp(-b * x)) + c
# This models heating: starts at c, approaches a + c asymptotically
def exp_growth(x, a, b, c):
    return a * (1 - np.exp(-b * x)) + c


# Inverse function: given temperature, return time in seconds
def exp_growth_inverse(temp, a, b, c):
    """
    Solve for x: temp = a * (1 - exp(-b * x)) + c
    => temp - c = a * (1 - exp(-b * x))
    => (temp - c) / a = 1 - exp(-b * x)
    => exp(-b * x) = 1 - (temp - c) / a
    => -b * x = ln(1 - (temp - c) / a)
    => x = -ln(1 - (temp - c) / a) / b
    """
    if a == 0 or b == 0:
        return None
    arg = 1 - (temp - c) / a
    if arg <= 0:
        return None  # Invalid for logarithm (temperature beyond asymptote)
    return -np.log(arg) / b


def get_brew_time(target_temp, popt):
    """
    Get the time in seconds required to reach a target temperature.
    
    Args:
        target_temp: Target temperature in °C
        popt: Fitted parameters (a, b, c) from curve_fit
        
    Returns:
        Time in seconds to reach target temperature, or None if unreachable
    """
    if popt is None:
        return None
    time_seconds = exp_growth_inverse(target_temp, *popt)
    if time_seconds is not None and time_seconds >= 0:
        return time_seconds
    return None


def fit_brew_data(data_path):
    """
    Load and fit the brew data, returning the fitted parameters.
    
    Returns:
        tuple: (popt, x_seconds, y, fit_success) where popt are the fitted parameters
    """
    x, y = parse_data_file(data_path)
    x_seconds = np.array(parse_time_to_seconds(x))
    y = np.array(y)
    
    # Initial guess for parameters: a (amplitude), b (rate), c (starting temp)
    guess = [y[-1] - y[0], 0.01, y[0]]
    
    try:
        popt, pcov = curve_fit(exp_growth, x_seconds, y, p0=guess, maxfev=10000)
        return popt, x_seconds, y, True
    except Exception as e:
        print(f"Fit failed: {e}")
        return None, x_seconds, y, False


def create_plots(popt, x_seconds, y, fit_success, output_dir):
    """Create and save the brew time plots."""
    
    if fit_success:
        y_fit = exp_growth(x_seconds, *popt)
        fit_label = f"Fit: T = {popt[0]:.2f} * (1 - exp(-{popt[1]:.6f} * t)) + {popt[2]:.2f}"
    else:
        y_fit = None
        fit_label = "Fit failed"
    
    # Create two subplots: Temperature vs Time and Time vs Temperature
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6), dpi=150)
    
    # Convert to minutes for display
    x_minutes = x_seconds / 60
    
    # LEFT PLOT: Temperature vs Time
    ax1.plot(x_minutes, y, marker='o', markersize=2, color='royalblue', label='Measured', alpha=0.5)
    if y_fit is not None:
        ax1.plot(x_minutes, y_fit, '--', linewidth=2, color='red', label=fit_label)
    ax1.set_xlabel('Time [minutes]')
    ax1.set_ylabel('Temperature [°C]')
    ax1.set_title('Brew Temperature vs Time')
    ax1.grid(True)
    ax1.legend()
    
    # RIGHT PLOT: Time vs Temperature (inverse function)
    if fit_success and popt is not None:
        # Create temperature range for inverse plot
        temp_min = popt[2] + 1  # Slightly above starting temperature
        temp_max = popt[0] + popt[2] - 1  # Slightly below asymptote
        temp_range = np.linspace(temp_min, min(temp_max, 100), 100)
        
        # Calculate time for each temperature (in minutes)
        time_for_temp = [exp_growth_inverse(t, *popt) for t in temp_range]
        time_for_temp = [(t / 60) if t is not None and t >= 0 else np.nan for t in time_for_temp]
        
        # Plot inverse function
        ax2.plot(temp_range, time_for_temp, '-', linewidth=2, color='green', label='Brew time function')
        
        # Plot measured data points (inverted axes)
        ax2.plot(y, x_minutes, 'o', markersize=2, color='royalblue', alpha=0.3, label='Measured data')
        
        ax2.set_xlabel('Target Temperature [°C]')
        ax2.set_ylabel('Time Required [minutes]')
        ax2.set_title('Time Required to Reach Target Temperature')
        ax2.grid(True)
        ax2.legend()
        
        # Add example annotations for common tea brewing temperatures
        example_temps = [60, 70, 80, 85, 90, 95]
        for temp in example_temps:
            if temp_min <= temp <= temp_max:
                time_needed = exp_growth_inverse(temp, *popt)
                if time_needed is not None and time_needed >= 0:
                    time_min = time_needed / 60
                    ax2.plot(temp, time_min, 'ro', markersize=6)
                    ax2.annotate(f'{temp}°C: {time_min:.1f}min ({time_needed:.0f}s)', 
                               xy=(temp, time_min), 
                               xytext=(5, 5), textcoords='offset points',
                               fontsize=8, color='darkred')
    else:
        ax2.text(0.5, 0.5, 'Inverse function unavailable\n(fit failed)', 
                ha='center', va='center', transform=ax2.transAxes, fontsize=14)
        ax2.set_xlabel('Target Temperature [°C]')
        ax2.set_ylabel('Time Required [minutes]')
        ax2.set_title('Time Required to Reach Target Temperature')
    
    plt.tight_layout()
    output_path = os.path.join(output_dir, 'brew_time_plot.png')
    plt.savefig(output_path)
    plt.close()
    print(f'Graph saved to {output_path}')
    
    return output_path


def print_brew_times(popt):
    """Print brew times for common tea temperatures."""
    if popt is None:
        print("Cannot calculate brew times - fit failed")
        return
    
    print("\n" + "=" * 60)
    print("BREW TIME LOOKUP TABLE")
    print("=" * 60)
    print(f"{'Temperature':^15} | {'Time (seconds)':^15} | {'Time (min:sec)':^15}")
    print("-" * 60)
    
    for temp in range(30, 101, 5):
        time_sec = get_brew_time(temp, popt)
        if time_sec is not None:
            minutes = int(time_sec // 60)
            seconds = int(time_sec % 60)
            print(f"{temp:^15}°C | {time_sec:^15.1f} | {minutes:>6}:{seconds:02d}")
        else:
            print(f"{temp:^15}°C | {'unreachable':^15} | {'N/A':^15}")
    
    print("=" * 60)
    print(f"\nFitted equation: T(t) = {popt[0]:.4f} * (1 - exp(-{popt[1]:.6f} * t)) + {popt[2]:.4f}")
    print(f"Inverse: t(T) = -ln(1 - (T - {popt[2]:.4f}) / {popt[0]:.4f}) / {popt[1]:.6f}")
    print(f"\nStarting temperature: {popt[2]:.2f}°C")
    print(f"Maximum reachable temperature: {popt[0] + popt[2]:.2f}°C")
    print("=" * 60)


if __name__ == "__main__":
    # Fit the data
    popt, x_seconds, y, fit_success = fit_brew_data(data_path)
    
    # Create plots
    create_plots(popt, x_seconds, y, fit_success, graphs_dir)
    
    # Print brew time table
    print_brew_times(popt)
    
    # Interactive example
    print("\n--- Example Usage ---")
    target = 85
    time_sec = get_brew_time(target, popt)
    if time_sec:
        print(f"To reach {target}°C: {time_sec:.1f} seconds ({time_sec/60:.2f} minutes)")
