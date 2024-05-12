import pandas as pd
import ast
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


file_path = '/home/s/dipl/src/hector_quadrotor/controller/trajectory_with_range_16_07_12_05.txt'

# Read the data
with open(file_path, 'r') as file:
    data = file.readlines()

# Parse the tuples from the file
data = [ast.literal_eval(line.strip()) for line in data]

# Create a DataFrame
df = pd.DataFrame(data, columns=['X', 'Y', 'Z', 'Range', 'Time'])

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plotting the trajectory
ax.scatter(df['X'], df['Y'], df['Z'], c='r', marker='o')
ax.set_xlabel('X Coordinate')
ax.set_ylabel('Y Coordinate')
ax.set_zlabel('Z Coordinate')

plt.title('3D Trajectory Plot')
plt.show()


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plotting the trajectory
ax.scatter(df['X'], df['Y'], df['Z'] - df['Range'], c='r', marker='o')
ax.set_xlabel('X Coordinate')
ax.set_ylabel('Y Coordinate')
ax.set_zlabel('Z Coordinate')

plt.title('3D Trajectory Plot')
plt.show()



plt.figure()
plt.plot(df['Time'], df['Z'], label='Z Coordinate over Time')
plt.plot(df['Time'], df['Range'], label='Range over Time', color='r')
plt.xlabel('Time')
plt.ylabel('Z Coordinate')
plt.title('Z Coordinate vs. Time')
plt.grid(True, color='gray', linestyle='--', linewidth=0.5)
plt.legend()
plt.show()


plt.figure()
plt.plot(df['Time'], df['X'], label='X Coordinate over Time')
plt.xlabel('Time')
plt.ylabel('X Coordinate')
plt.title('X Coordinate vs. Time')
plt.legend()
plt.grid(True, color='gray', linestyle='--', linewidth=0.5)
plt.show()

plt.figure()
plt.plot(df['Time'], df['Y'], label='Y Coordinate over Time')
plt.xlabel('Time')
plt.ylabel('Y Coordinate')
plt.title('Y Coordinate vs. Time')
plt.legend()
plt.grid(True, color='gray', linestyle='--', linewidth=0.5)
plt.show()