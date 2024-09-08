import pandas as pd
import matplotlib.pyplot as plt

force_control_df = pd.read_csv("force_vs_time_fc.csv", header=None, names=['time', 'force'])
impedance_control_df = pd.read_csv("force_vs_time_ic.csv", header=None, names=['time', 'force'])

# Filter for the first 10 seconds
force_control_df_filtered = force_control_df[force_control_df['time'] <= 10]
impedance_control_df_filtered = impedance_control_df[impedance_control_df['time'] <= 10]

# Plotting with updated labels and legends
plt.figure(figsize=(10, 6))

plt.plot(force_control_df_filtered['time'], force_control_df_filtered['force'], label='Force Control')
plt.plot(impedance_control_df_filtered['time'], impedance_control_df_filtered['force'], label='Impedance Control')

plt.xlabel("Time (s)")
plt.ylabel("Force (N)")
plt.title("Force Control vs Impedance Control Comparison")
plt.grid(True)
plt.legend()

plt.show()

