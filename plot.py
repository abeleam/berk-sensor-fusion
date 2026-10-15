import pandas as pd
import matplotlib.pyplot as plt

# load the output our filter produced
out = pd.read_csv('output_dev.csv')

# two plots stacked vertically
fig, axes = plt.subplots(2, 1, figsize=(12, 8))

# top plot: z acceleration over time
# az is the most interesting axis because the vehicle goes up
axes[0].plot(out['t_s'], out['ax_mps2'], label='ax')
axes[0].plot(out['t_s'], out['ay_mps2'], label='ay')
axes[0].plot(out['t_s'], out['az_mps2'], label='az')
axes[0].set_title('Estimated Acceleration')
axes[0].set_xlabel('Time (s)')
axes[0].set_ylabel('m/s^2')
axes[0].legend()
axes[0].grid(True)

# bottom plot: altitude over time
# should go up then come back down
axes[1].plot(out['t_s'], out['altitude_m'], label='altitude', color='orange')
axes[1].set_title('Estimated Altitude')
axes[1].set_xlabel('Time (s)')
axes[1].set_ylabel('meters')
axes[1].legend()
axes[1].grid(True)

plt.tight_layout()
plt.savefig('plots.png') # saves the image so you can attach it to your submission
plt.show()
print("Plot saved to plots.png")