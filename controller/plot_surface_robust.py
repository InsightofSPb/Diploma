import scipy.spatial as ss
import numpy as np
import pandas as pd
import ast

file_path = '/home/s/dipl/src/hector_quadrotor/controller/results/trajectory_with_range_16_07_12_05.txt'

# Read the data
with open(file_path, 'r') as file:
    data = file.readlines()


# Parse the tuples from the file
data = [ast.literal_eval(line.strip()) for line in data]

# Create a DataFrame
df = pd.DataFrame(data, columns=['X', 'Y', 'Z', 'Range', 'Time'])

grid_x, grid_y = np.mgrid[min(df['X']):max(df['X']):1000j, min(df['Y']):max(df['Y']):1000j]

df['dif'] = df['Z'] - df['Range']

result_df = df[['X', 'Y', 'dif']]
# Convert the DataFrame to a NumPy array
points = result_df.to_numpy()

# Create the convex hull
hull = ss.ConvexHull(points)

# Print the volume enclosed by the points
print('Volume inside points is:', hull.volume)