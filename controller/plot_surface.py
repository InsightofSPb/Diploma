import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import ast
from matplotlib import cm
from matplotlib.ticker import LinearLocator
from scipy.interpolate import griddata

file_path = '/home/s/dipl/src/hector_quadrotor/controller/trajectory_with_range_18_42_12_05.txt'

# Read the data
with open(file_path, 'r') as file:
    data = file.readlines()


# Parse the tuples from the file
data = [ast.literal_eval(line.strip()) for line in data]

# Create a DataFrame
df = pd.DataFrame(data, columns=['X', 'Y', 'Z', 'Range', 'Time'])

grid_x, grid_y = np.mgrid[min(df['X']):max(df['X']):1000j, min(df['Y']):max(df['Y']):1000j]

z_data = df['Z'] - df['Range']
# Interpolate the data
grid_z = griddata((df['X'], df['Y']), z_data, (grid_x, grid_y), method='linear', fill_value=0)

integrated_along_y = np.trapz(grid_z, x=grid_y, axis=1)

# Then integrate the result over the other axis
total_volume = np.trapz(integrated_along_y, x=grid_x[:, 0])

print("Total volume under the surface:", total_volume)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the surface
surf = ax.plot_surface(grid_x, grid_y, grid_z, cmap='viridis', edgecolor='none')

# Add a color bar which maps values to colors
cbar = fig.colorbar(surf, ax=ax, shrink=0.5, aspect=5)
cbar.set_label('Range')

ax.set_xlabel('X Coordinate')
ax.set_ylabel('Y Coordinate')
ax.set_zlabel('Range')

plt.title('3D Surface Plot of Range')
plt.show()