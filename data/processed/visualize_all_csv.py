
import pandas as pd
import matplotlib.pyplot as plt

# Define file paths
processed_dir = '/home/hannibal/Mandol_ws/data/processed'
left_lane_path = f'{processed_dir}/왼쪽 차선 utm.csv'
right_lane_path = f'{processed_dir}/오른쪽 차선 utm.csv'
# gps_data_path = f'{processed_dir}/이전대회 GPS 데이터 (기본+평행주차)_direction.csv'
gps_data_path = f'{processed_dir}/3_T_left_P_right.csv'

# Load the data
left_lane_data = pd.read_csv(left_lane_path)
right_lane_data = pd.read_csv(right_lane_path)
gps_data = pd.read_csv(gps_data_path)

# If 'direction' column is missing, assume all are 0
if 'direction' not in gps_data.columns:
    gps_data['direction'] = 0

# Create a plot
plt.figure(figsize=(12, 12))

# Plot left and right lanes
plt.plot(left_lane_data['X(E/m)'], left_lane_data['Y(N/m)'], color='orange', linestyle='-', linewidth=2, label='Left Lane')
plt.plot(right_lane_data['X(E/m)'], right_lane_data['Y(N/m)'], color='green', linestyle='-', linewidth=2, label='Right Lane')

# Plot the main GPS path
plt.plot(gps_data['X(E/m)'], gps_data['Y(N/m)'], color='black', linestyle='-', linewidth=1, label='Path')

# Define colors for each direction
colors = {1: 'red', 0: 'black', -1: 'blue'}

# Plot the colored points for the main GPS path
for direction, color in colors.items():
    subset = gps_data[gps_data['direction'] == direction]
    plt.scatter(subset['X(E/m)'], subset['Y(N/m)'], c=color, label=f'Direction {direction}', s=20)

# Set labels and title
plt.xlabel('X(E/m)')
plt.ylabel('Y(N/m)')
plt.title('Combined GPS Data Visualization')
plt.legend()
plt.grid(True)
plt.axis('equal')

# Show the plot
plt.show()
