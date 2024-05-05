import Controller
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# Create an instance for class.
boat_controller = Controller.BoatController()

# Load the normalized motor speed data from CSV files.
norm_left_motor_speed = boat_controller.load_csv("norm_left_motor_speed.csv")
norm_right_motor_speed = boat_controller.load_csv("norm_right_motor_speed.csv")

# Distance and heading of the boat.
x = np.linspace(-1, 1, 51)   # Generate 51 evenly spaced values from -1 to 1 for x-axis.
y = np.linspace(0, 1, 51)    # Generate 51 evenly spaced values from 0 to 1 for y-axis.
[x, y] = np.meshgrid(x, y)

# Create two 3D plot.
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the surface with intensity levels.
surf = ax.plot_surface(x, y, norm_left_motor_speed, cmap='viridis', edgecolor='none')
#surf = ax.plot_surface(x, y, norm_right_motor_speed, cmap='viridis', edgecolor='none')

# Add colour bar.
fig.colorbar(surf, ax=ax, shrink=0.5, aspect=5)

# Set labels and title.
ax.set_xlabel('Heading (Degree)')
ax.set_ylabel('Distance (m)')
ax.set_zlabel('Speed (PWM)')
ax.set_title('3D Map with Intensity Levels for Motor Speed')

# Plot the 3D graph.
plt.show()
