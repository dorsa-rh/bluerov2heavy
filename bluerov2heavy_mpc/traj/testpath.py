#--------------------------------------
# Straight-line trajectory through user-defined waypoints
#--------------------------------------

import numpy as np

# --- Waypoints (x, y, z) ---
waypoints = np.array([
    (0.0,   0.0,  -14.0),
    (2.87,  0.0,  -14.0),
    (2.87, -8.2,  -14.0),
    (-4.63,-8.2,  -14.0),
    (-4.63,-20.2, -14.0),
    (-7.63,-20.2, -14.0)
])

sample_time = 0.05      # seconds
v = 1                 # speed (m/s)
stationary_time = 15.0  # stationary segment at start

# Control placeholders
u1_placeholder = 0.0
u2_placeholder = 0.0
u3_placeholder = 0.0
u4_placeholder = 0.0
u5_placeholder = 0.0
u6_placeholder = 0.0
# Compute segment distances (in XY only)
deltas = waypoints[1:, :2] - waypoints[:-1, :2]
seg_lengths = np.linalg.norm(deltas, axis=1)

# Compute time for each straight segment
seg_times = seg_lengths / v
seg_times = np.maximum(seg_times, 0.1)  # minimum segment time

# Build trajectory list
traj_list = []

for i in range(len(seg_lengths)):
    wp_start = waypoints[i]
    wp_end = waypoints[i+1]

    t_seg = np.arange(0, seg_times[i] + sample_time, sample_time)

    # Linear interpolation
    alpha = t_seg / seg_times[i]
    x = wp_start[0] + alpha * (wp_end[0] - wp_start[0])
    y = wp_start[1] + alpha * (wp_end[1] - wp_start[1])
    z = wp_start[2]  # constant -15

    # Direction of straight line for yaw
    dx = (wp_end[0] - wp_start[0])
    dy = (wp_end[1] - wp_start[1])
    psi = np.arctan2(dy, dx)

    # Velocities (constant direction)
    u = v * np.cos(psi)
    v_y = v * np.sin(psi)

    # Build segment trajectory
    seg_traj = np.zeros((len(t_seg), 18))

    seg_traj[:, 0] = x
    seg_traj[:, 1] = y
    seg_traj[:, 2] = z
    seg_traj[:, 5] = psi   # yaw
    seg_traj[:, 6] = u     # u velocity
    seg_traj[:, 7] = v_y   # v velocity
    seg_traj[:, 14] = u3_placeholder

    traj_list.append(seg_traj)

# Concatenate all straight segments
traj = np.vstack(traj_list)

# --- Add stationary initial segment ---
n_sta = int(stationary_time / sample_time) + 1
sta = np.zeros((n_sta, 18))
sta[:, 0] = waypoints[0, 0]
sta[:, 1] = waypoints[0, 1]
sta[:, 2] = waypoints[0, 2]
sta[:, 5] = traj[0, 5]  # yaw

traj_full = np.vstack((sta, traj))

# Save file
np.savetxt("test.txt", traj_full, fmt="%.6f")

print("Straight-line trajectory saved to test.txt")
