#--------------------------------------
#Generate reference trajectory for NMPC
#--------------------------------------

import numpy as np
import math
#import tf.transformations as tf


dt = 0.02
T  = 240.0
N  = int(T / dt) + 1

r = 2.0          # radius [m]
v_nom = 0.5      # nominal speed [m/s]

x0, y0, z0 = 0.0, 0.0, -5.0

# Time
t = np.linspace(0, T, N)

# Smooth speed profile (raised cosine)
T_ramp = 10.0
s = np.ones_like(t)
idx = t < T_ramp
s[idx] = 0.5 * (1 - np.cos(np.pi * t[idx] / T_ramp))

v = v_nom * s

# Angular rate
omega = v / r
theta = np.cumsum(omega) * dt

# Allocate trajectory
traj = np.zeros((N, 20))

# Position
traj[:,0] = x0 - r * np.cos(theta)
traj[:,1] = y0 - r * np.sin(theta)
traj[:,2] = z0

# Orientation (yaw aligned with velocity)
traj[:,3] = 0.5            # phi
traj[:,4] = 0.0            # theta
#traj[:,5] = theta - np.pi/2
traj[:,5] = 0.0            # psi (fixed yaw for simplicity)

# Body-frame velocities (calculated from position derivatives)
# dx/dt and dy/dt in world frame
dx = np.zeros(N)
dy = np.zeros(N)
for i in range(1, N):
    dx[i] = (traj[i,0] - traj[i-1,0]) / dt
    dy[i] = (traj[i,1] - traj[i-1,1]) / dt

# Convert world frame velocities to body frame
# Since yaw is fixed at 0, body frame = world frame
traj[:,6] = dx                          # u (surge)
traj[:,7] = dy                          # v (sway)
traj[:,8] = 0.0                         # w (heave)

# Angular rates
traj[:,9]  = 0.0           # p
traj[:,10] = 0.0           # q
traj[:,11] = 0         # r #omega

# Control references (zero wrench)
traj[:,12:20] = 0.0

# write to txt
np.savetxt('circle.txt',traj,fmt='%f')
