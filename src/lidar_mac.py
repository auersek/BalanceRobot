import socket
import struct
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

# --- Constants ---
UDP_PORT = 5005
PACKET_LENGTH = 47
MEASUREMENT_LENGTH = 12
MESSAGE_FORMAT = "<xBHH" + "HB" * MEASUREMENT_LENGTH + "HHB"

PLOT_MAX_RANGE = 4.0
PLOT_CONFIDENCE_COLOUR_MAP = "bwr_r"

# --- Setup UDP socket ---
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", UDP_PORT))
sock.settimeout(1.0)

# --- Setup Matplotlib plot ---
plt.ion()
plt.rcParams['figure.figsize'] = [10, 10]
plt.rcParams['lines.markersize'] = 2.0

graph = plt.scatter([], [], c=[], marker=".", vmin=0, vmax=255, cmap=PLOT_CONFIDENCE_COLOUR_MAP)
plt.xlim(-PLOT_MAX_RANGE, PLOT_MAX_RANGE)
plt.ylim(-PLOT_MAX_RANGE, PLOT_MAX_RANGE)

# --- Buffers for full sweep ---
x_buffer = deque()
y_buffer = deque()
confidence_buffer = deque()
angle_coverage = 0.0  # Track angular sweep progress

# --- Main Loop ---
while True:
    try:
        data, _ = sock.recvfrom(1024)
        if len(data) != PACKET_LENGTH:
            continue

        # Unpack LiDAR packet
        _, speed, start_angle, *pos_data, stop_angle, timestamp, crc = struct.unpack(MESSAGE_FORMAT, data)
        start_angle /= 100.0
        stop_angle /= 100.0
        if stop_angle < start_angle:
            stop_angle += 360.0
        step_size = (stop_angle - start_angle) / (MEASUREMENT_LENGTH - 1)

        angle = np.array([start_angle + step_size * i for i in range(MEASUREMENT_LENGTH)])
        distance = np.array(pos_data[0::2])
        confidence = np.array(pos_data[1::2])

        # Convert to Cartesian coordinates
        x = np.sin(np.radians(angle)) * (distance / 1000.0)
        y = np.cos(np.radians(angle)) * (distance / 1000.0)

        # Append to sweep buffer
        x_buffer.extend(x)
        y_buffer.extend(y)
        confidence_buffer.extend(confidence)
        angle_coverage += (stop_angle - start_angle)

        # Once ~360° covered, plot the sweep
        if angle_coverage >= 360.0:
            graph.remove()
            graph = plt.scatter(x_buffer, y_buffer, c=confidence_buffer, marker=".", vmin=0, vmax=255, cmap=PLOT_CONFIDENCE_COLOUR_MAP)
            plt.pause(0.001)

            # Reset for next full sweep
            x_buffer.clear()
            y_buffer.clear()
            confidence_buffer.clear()
            angle_coverage = 0.0

    except socket.timeout:
        continue
