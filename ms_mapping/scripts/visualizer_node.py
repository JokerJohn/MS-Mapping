#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2

# Initialize lists to store the data over time
time_data = []
ws_data = []
voxel_data = []
keyframe_data = []
# Define your threshold here
wd_threshold = 3.5
# Plotting setup
fig, axs = plt.subplots(1, 3, figsize=(24, 6))


# Function to update the plot with new data
def update_plot(frame):
    # Clear previous plots
    for ax in axs:
        ax.clear()
    # Time relative to first data point
    relative_time = np.array(time_data) - time_data[0] if time_data else np.array([0])
    # Plot trajectory with keyframes
    axs[0].plot(ws_data, voxel_data, color='blue', label='Trajectory')
    axs[0].scatter(time_data[0], voxel_data[0], color='lime', marker='o', s=100, label='Start')
    axs[0].scatter(time_data[-1], voxel_data[-1], color='blue', marker='X', s=100, label='End')
    axs[0].scatter(np.array(time_data)[np.array(keyframe_data) == 1],
                   np.array(voxel_data)[np.array(keyframe_data) == 1],
                   color='red', s=10, label='Keyframes')
    axs[0].set_aspect('equal', 'datalim')
    axs[0].set_title('XY Trajectory with Keyframes')
    axs[0].set_xlabel('WS Distance')
    axs[0].set_ylabel('Voxel Count')
    axs[0].legend()
    # Plot voxel count over time
    axs[1].plot(relative_time, voxel_data, 'green', label='Voxel Number')
    axs[1].scatter(relative_time[np.array(keyframe_data) == 1],
                   np.array(voxel_data)[np.array(keyframe_data) == 1],
                   color='purple', s=10, label='Keyframes')
    axs[1].set_title('Voxel Number Over Time')
    axs[1].set_xlabel('Time [s]')
    axs[1].set_ylabel('Voxel Count')
    axs[1].legend()
    # Plot WS distance over time with threshold and keyframe marking
    axs[2].plot(relative_time, ws_data, 'darkmagenta', label='WS Distance')
    axs[2].scatter(relative_time[np.array(keyframe_data) == 1],
                   np.array(ws_data)[np.array(keyframe_data) == 1],
                   color='red', s=50, label='Above Threshold')
    axs[2].axhline(wd_threshold, color='blue', linestyle='--', label='WS Threshold')
    axs[2].set_title('Wasserstein Distance Over Time')
    axs[2].set_xlabel('Time [s]')
    axs[2].set_ylabel('WS [m]')
    axs[2].legend()
    plt.tight_layout()


# ROS Subscriber callbacks
def ws_callback(msg):
    # Append the WS distance data
    ws_data.append(msg.data)
    time_data.append(rospy.get_time())  # Assuming time is in seconds
    # Check if WS is above threshold for keyframe
    if msg.data > wd_threshold:
        keyframe_data.append(1)  # Mark as keyframe
    else:
        keyframe_data.append(0)  # Not a keyframe


def voxel_callback(msg):
    # Append voxel number data (assuming msg contains a field for voxel count)
    voxel_data.append(msg.data)


# Initialize ROS node
def init_ros_node():
    rospy.init_node('real_time_visualizer', anonymous=True)
    # Subscribers to receive data (assuming relevant messages are being published)
    rospy.Subscriber('/ws_distance', Float32, ws_callback)
    rospy.Subscriber('/voxel_number', Float32, voxel_callback)
    # Animation function to update the plot every 100ms
    ani = FuncAnimation(fig, update_plot, interval=100)
    # Display the plot
    plt.show()
    rospy.spin()


if __name__ == "__main__":
    init_ros_node()
